"""Re-fit the pursuit down-camera back-projection from ground-truth calibration data.

The monocular ground back-projection in sim/scripts/vision_detect.py (`detect_world`) had a
systematic ~4 m bias -> the swarm followed the target offset. This fits the camera->body
rotation `_R_BODY_CAM` (and the horizontal FOV) that best maps each logged (pixel, drone pose)
to the KNOWN true target position, over the /tmp/calib.csv samples collected with CALIB=1.

Run a data collection first:  GUI=0 VISION=1 CALIB=1 run_px4_swarm.sh swarm_autonomy_city 3
then:                         python3 experiments/fit_camera_extrinsic.py
Prints BEFORE/AFTER error and the matrix to paste into vision_detect._R_BODY_CAM.
"""
import csv
import math
import sys

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R

CSV = sys.argv[1] if len(sys.argv) > 1 else "/tmp/calib_saved.csv"
T_NED_ENU = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])

rows = []
for r in csv.DictReader(open(CSV)):
    rows.append((float(r["cx"]), float(r["cy"]), int(r["W"]), int(r["H"]),
                 float(r["dE"]), float(r["dN"]), float(r["alt"]), float(r["roll"]),
                 float(r["pitch"]), float(r["yaw"]), float(r["tgtE"]), float(r["tgtN"])))
print(f"{len(rows)} calibration samples from {CSV}")


def errors(params):
    Rbc = R.from_rotvec(params[:3]).as_matrix()
    hfov = params[3]
    out = []
    for (cx, cy, W, H, dE, dN, alt, roll, pitch, yaw, tE, tN) in rows:
        vfov = 2 * math.atan(math.tan(hfov / 2) * H / W)
        nx = (cx - W / 2) / (W / 2)
        ny = (cy - H / 2) / (H / 2)
        d = np.array([nx * math.tan(hfov / 2), ny * math.tan(vfov / 2), 1.0])
        d /= np.linalg.norm(d)
        Rnb = R.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
        denu = T_NED_ENU @ (Rnb @ (Rbc @ d))
        if denu[2] >= -1e-3:
            out += [5.0, 5.0]
            continue
        s = alt / (-denu[2])
        out += [dE + s * denu[0] - tE, dN + s * denu[1] - tN]
    return np.array(out)


def rms(params):
    e = errors(params).reshape(-1, 2)
    d = np.hypot(e[:, 0], e[:, 1])
    return d.mean(), np.sqrt(np.mean(d ** 2)), np.median(d)


cur = np.concatenate([R.from_matrix([[0, -1, 0], [1, 0, 0], [0, 0, 1]]).as_rotvec(), [1.74]])
print("BEFORE: mean=%.2fm  rms=%.2fm  median=%.2fm" % rms(cur))

sol = least_squares(errors, cur, method="lm", max_nfev=4000)
print("AFTER:  mean=%.2fm  rms=%.2fm  median=%.2fm" % rms(sol.x))
Rbc = R.from_rotvec(sol.x[:3]).as_matrix()
print(f"\nfitted hfov = {sol.x[3]:.4f} rad  (was 1.74)")
print("fitted _R_BODY_CAM = [")
for row in Rbc:
    print("    [%.5f, %.5f, %.5f]," % (row[0], row[1], row[2]))
print("]")
