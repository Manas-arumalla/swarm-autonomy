"""Consensus-Based Bundle Algorithm (CBBA) — decentralized, convergent task allocation.

Reference: Choi, Brunet & How, "Consensus-Based Decentralized Auctions for Robust
Task Allocation," IEEE T-RO 2009. Used to assign scout / blocker / interceptor
roles without a central node.

This module is deliberately ROS-free and deterministic so it can be unit-tested
offline and reused from headless simulation environments unchanged. The ROS
node (``cbba_node``) wraps it, exchanging the (y, z) winning-bid vectors through
the comms middleware as :class:`swarm_autonomy_msgs.msg.TaskBid` messages.

Two phases per iteration:

1. **Bundle building** — each agent greedily appends the task with the highest
   *marginal* score to its bundle until no task improves its path or the bundle
   is full. Scores must be diminishing-marginal-gain (DMG) for CBBA's
   convergence/optimality guarantees to hold.
2. **Consensus** — agents exchange winning bids ``y`` (best score per task) and
   winners ``z``; an agent that has been outbid releases that task and every
   task added after it in the bundle.

With synchronous all-to-all comms this converges in at most ``n_agents`` rounds
to a conflict-free assignment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Sequence

# A score function maps (agent_id, path_so_far, candidate_task) -> marginal score.
# It MUST be diminishing-marginal-gain for CBBA guarantees to hold.
ScoreFn = Callable[[int, Sequence[int], int], float]


@dataclass
class Agent:
    agent_id: int
    n_tasks: int
    max_bundle: int = 1
    # CBBA state vectors, indexed by task id.
    y: list[float] = field(default_factory=list)   # best known winning score per task
    z: list[int] = field(default_factory=list)      # believed winner per task (-1 = none)
    bundle: list[int] = field(default_factory=list)  # tasks in insertion order
    path: list[int] = field(default_factory=list)    # ordered path (== bundle here)

    def __post_init__(self) -> None:
        if not self.y:
            self.y = [0.0] * self.n_tasks
        if not self.z:
            self.z = [-1] * self.n_tasks

    # --- phase 1 -----------------------------------------------------------
    def build_bundle(self, score_fn: ScoreFn) -> None:
        while len(self.bundle) < self.max_bundle:
            best_task, best_score = -1, 0.0
            for j in range(self.n_tasks):
                if j in self.bundle:
                    continue
                c = score_fn(self.agent_id, self.path, j)
                # Only claim tasks we can actually win (beat the current holder).
                if c > self.y[j] and c > best_score:
                    best_task, best_score = j, c
            if best_task < 0:
                break
            self.bundle.append(best_task)
            self.path.append(best_task)
            self.y[best_task] = best_score
            self.z[best_task] = self.agent_id

    # --- phase 2 -----------------------------------------------------------
    def update_from(self, other: "Agent", tie_break: dict[int, float]) -> bool:
        """Merge neighbour ``other``'s (y, z); return True if our state changed.

        ``tie_break`` maps agent_id -> deterministic priority (higher wins ties),
        standing in for the timestamp vector of the full async algorithm.
        """
        changed = False
        outbid_from = self.n_tasks  # earliest bundle position we lose
        for j in range(self.n_tasks):
            their_y, their_z = other.y[j], other.z[j]
            if their_z == -1:
                continue
            if their_z == self.agent_id and j not in self.bundle:
                # Neighbour still believes WE hold j, but we have released it. Canonical CBBA's
                # *reset* action: clear the belief so j can be re-bid, instead of re-adopting a
                # phantom self-win (which would leave j permanently "won" yet in no bundle).
                if self.z[j] != -1 or self.y[j] != 0.0:
                    self.z[j] = -1
                    self.y[j] = 0.0
                    changed = True
                continue
            better = their_y > self.y[j] + 1e-12
            tie = abs(their_y - self.y[j]) <= 1e-12 and self.z[j] != their_z and \
                tie_break.get(their_z, their_z) > tie_break.get(self.z[j], self.z[j])
            if better or tie:
                if self.z[j] == self.agent_id and j in self.bundle:
                    outbid_from = min(outbid_from, self.bundle.index(j))
                self.y[j] = their_y
                self.z[j] = their_z
                changed = True

        # Release the outbid task and everything appended after it.
        if outbid_from < len(self.bundle):
            for k in self.bundle[outbid_from + 1:]:
                # Re-open tasks we no longer hold (unless someone else now holds them).
                if self.z[k] == self.agent_id:
                    self.z[k] = -1
                    self.y[k] = 0.0
            released = self.bundle[outbid_from:]
            self.bundle = self.bundle[:outbid_from]
            self.path = self.path[:outbid_from]
            for k in released:
                if self.z[k] == self.agent_id:
                    self.z[k] = -1
                    self.y[k] = 0.0
            changed = True
        return changed


def run_cbba(
    n_agents: int,
    n_tasks: int,
    score_fn: ScoreFn,
    max_bundle: int = 1,
    comm_graph: dict[int, list[int]] | None = None,
    max_rounds: int | None = None,
) -> tuple[dict[int, list[int]], list[int]]:
    """Run CBBA to convergence (synchronous rounds over ``comm_graph``).

    Returns ``(assignment, z)`` where ``assignment[agent] = [task, ...]`` and
    ``z[task] = winning_agent`` (-1 if unassigned).
    """
    agents = {a: Agent(a, n_tasks, max_bundle) for a in range(n_agents)}
    if comm_graph is None:  # fully connected
        comm_graph = {a: [b for b in range(n_agents) if b != a] for a in range(n_agents)}
    tie_break = {a: float(a) for a in range(n_agents)}  # deterministic priority
    rounds = max_rounds if max_rounds is not None else n_agents * max_bundle + 2

    for _ in range(rounds):
        for a in agents.values():
            a.build_bundle(score_fn)
        snapshot = {a: Agent(a, n_tasks, max_bundle, list(ag.y), list(ag.z),
                             list(ag.bundle), list(ag.path))
                    for a, ag in agents.items()}
        changed = False
        for a, ag in agents.items():
            for nb in comm_graph[a]:
                if ag.update_from(snapshot[nb], tie_break):
                    changed = True
        if not changed:
            break

    assignment = {a: list(ag.bundle) for a, ag in agents.items()}
    # Derive the winner vector from the actual (conflict-free) assignment rather than from one
    # agent's belief vector, which can disagree on a disconnected or not-yet-converged graph.
    z = [-1] * n_tasks
    for a, tasks in assignment.items():
        for t in tasks:
            z[t] = a
    return assignment, z
