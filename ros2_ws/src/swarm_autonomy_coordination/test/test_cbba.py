"""Unit tests for the CBBA allocator."""

from swarm_autonomy_coordination.cbba import run_cbba


def _conflict_free(assignment):
    seen = set()
    for tasks in assignment.values():
        for t in tasks:
            assert t not in seen, f"task {t} assigned twice"
            seen.add(t)
    return seen


def test_single_assignment_is_conflict_free():
    # 3 agents, 3 tasks, each agent strictly prefers a distinct task.
    prefs = {0: 0, 1: 1, 2: 2}

    def score_fn(agent, path, task):
        return 10.0 if prefs[agent] == task else 1.0

    assignment, z = run_cbba(3, 3, score_fn, max_bundle=1)
    _conflict_free(assignment)
    # Every agent should win its preferred task.
    for a, t in prefs.items():
        assert assignment[a] == [t]
        assert z[t] == a


def test_more_agents_than_tasks_leaves_some_idle():
    def score_fn(agent, path, task):
        return 5.0 - task  # everyone prefers task 0, then 1

    assignment, z = run_cbba(4, 2, score_fn, max_bundle=1)
    won = _conflict_free(assignment)
    assert len(won) == 2  # only two tasks exist
    idle = [a for a, t in assignment.items() if not t]
    assert len(idle) == 2


def test_bundle_allocation_distributes_tasks():
    # 2 agents, 4 tasks, diminishing marginal gain so bundles stay balanced.
    def score_fn(agent, path, task):
        return (4.0 - task) / (1.0 + len(path))  # DMG: later picks worth less

    assignment, z = run_cbba(2, 4, score_fn, max_bundle=4)
    won = _conflict_free(assignment)
    assert len(won) == 4  # all tasks allocated
    # Neither agent hoards everything.
    assert all(0 < len(t) < 4 for t in assignment.values())


def test_converges_on_sparse_comm_graph():
    # Line graph 0-1-2: information must propagate, but still converge.
    graph = {0: [1], 1: [0, 2], 2: [1]}
    prefs = {0: 0, 1: 0, 2: 0}  # contention on task 0

    def score_fn(agent, path, task):
        # agent 2 bids highest on the contested task 0
        base = {0: 1.0, 1: 2.0, 2: 3.0}[agent]
        return base if task == 0 else 0.1

    assignment, z = run_cbba(3, 3, score_fn, max_bundle=1, comm_graph=graph)
    _conflict_free(assignment)
    assert z[0] == 2  # highest bidder wins the contested task


def test_z_is_consistent_with_assignment():
    # The returned winner vector must always agree with the actual bundles (no phantom winners,
    # no winner for an unassigned task) across a range of configs and comm graphs.
    graphs = [None, {0: [1], 1: [0, 2], 2: [1]}, {0: [1, 2], 1: [0], 2: [0]}]
    for g in graphs:
        for mb in (1, 2):
            def score_fn(agent, path, task, _g=g):
                return (5.0 - task) / (1.0 + len(path)) + 0.1 * ((agent + task) % 3)

            n_ag = 3 if g is None else len(g)
            assignment, z = run_cbba(n_ag, 4, score_fn, max_bundle=mb, comm_graph=g)
            holder = {}
            for a, tasks in assignment.items():
                for t in tasks:
                    assert t not in holder, "task assigned twice"
                    holder[t] = a
            for t in range(4):
                assert z[t] == holder.get(t, -1)   # winner vector matches the bundles exactly
