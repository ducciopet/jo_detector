#!/usr/bin/env python3
"""
detector_eval_pipeline.py
─────────────────────────────────────────────────────────────────────────────
Valuta la classificazione Dynamic / Potentially-Dynamic di jo_detector
rispetto al ground truth Qualisys.

Topic usato: /perception/tracked_dynamic_obstacles (jo_msgs/ObstacleArray)
Status constants (Obstacle.msg):
    STATUS_STATIC              = 0
    STATUS_DYNAMIC             = 1
    STATUS_POTENTIALLY_DYNAMIC = 2
    STATUS_TRACKED             = 3

GT label derivato dalla velocità assoluta della persona in world frame (Qualisys):
    |v_persona| >  dyn_vel_thresh     →  GT = DYNAMIC
    |v_persona| <= static_vel_thresh  →  GT = STATIC
    altrimenti                        →  GT = POTENTIALLY_DYNAMIC

Per ogni esperimento vengono calcolati (per classe DYNAMIC, POTENTIALLY_DYNAMIC e STATIC):
    Precision, Recall, F1, Accuracy + detection rate complessiva.

Uso:
    python3 detector_eval_pipeline.py \\
        --detector_bag  experiment.mcap \\
        --gt_bag        qualisys.db3 \\
        --gt_topic      /rigid_bodies \\
        --person_body   PERSONA \\
        --detector_ros_version jazzy \\
        --gt_ros_version       humble \\
        --output_dir    ./results/exp1 \\
        [--detector_topic /perception/tracked_dynamic_obstacles] \\
        [--t_start 10.0] \\
        [--dyn_vel_thresh 0.15] \\
        [--match_dist_thresh 1.0] \\
        [--t_max_diff 0.1] \\
        [--vel_smooth_window 9] \\
        [--time_mode relative] \\
        [--gt_yaw 0.0 --gt_tx 0.0 --gt_ty 0.0 --gt_tz 0.0] \\
        [--auto_align_xy]
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError:
    print("[ERROR] rosbags non installato. Esegui: pip install rosbags --break-system-packages")
    sys.exit(1)

# ── Costanti status ostacoli ──────────────────────────────────────────────────
STATUS_STATIC              = 0
STATUS_DYNAMIC             = 1
STATUS_POTENTIALLY_DYNAMIC = 2
STATUS_TRACKED             = 3
NOT_DETECTED               = -1

GT_CLASSES = (STATUS_DYNAMIC, STATUS_POTENTIALLY_DYNAMIC, STATUS_STATIC)
PRED_CLASSES = (STATUS_DYNAMIC, STATUS_POTENTIALLY_DYNAMIC, STATUS_STATIC, NOT_DETECTED)
LABEL_MAP = {
    STATUS_DYNAMIC: 'DYNAMIC',
    STATUS_POTENTIALLY_DYNAMIC: 'POTENTIALLY_DYNAMIC',
    STATUS_STATIC: 'STATIC',
    STATUS_TRACKED: 'STATIC',  # TRACKED is not a classifier output; treated as STATIC
    NOT_DETECTED: 'NOT_DETECTED',
}

# ── Definizioni messaggi custom ───────────────────────────────────────────────
_MARKER_MSG = """\
int8 USE_NAME=0
int8 USE_INDEX=1
int8 USE_BOTH=2
int8 id_type
int32 marker_index
string marker_name
geometry_msgs/Point translation"""

_RIGID_BODY_MSG = """\
string rigid_body_name
Marker[] markers
geometry_msgs/Pose pose"""

_RIGID_BODIES_MSG = """\
std_msgs/Header header
uint32 frame_number
mocap4r2_msgs/RigidBody[] rigidbodies"""

_OBSTACLE_MSG = """\
std_msgs/Header header
uint32 track_id
uint32 track_age
uint8 STATUS_STATIC=0
uint8 STATUS_DYNAMIC=1
uint8 STATUS_POTENTIALLY_DYNAMIC=2
uint8 STATUS_TRACKED=3
uint8 status
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
geometry_msgs/Twist twist
geometry_msgs/Accel accel"""

_OBSTACLE_ARRAY_MSG = """\
std_msgs/Header header
jo_msgs/Obstacle[] obstacles"""

ROS_VERSION_MAP = {
    "humble": Stores.ROS2_HUMBLE,
    "jazzy":  Stores.ROS2_JAZZY,
    "iron":   Stores.ROS2_IRON,
    "foxy":   Stores.ROS2_FOXY,
}


def get_typestore_for_version(version: str, include_jo_msgs: bool = False,
                              include_mocap: bool = False):
    version = version.lower()
    if version not in ROS_VERSION_MAP:
        print(f"[WARN] Versione '{version}' non riconosciuta, uso jazzy.")
        version = "jazzy"
    ts = get_typestore(ROS_VERSION_MAP[version])
    add_types = {}
    if include_mocap:
        add_types.update(get_types_from_msg(_MARKER_MSG,       "mocap4r2_msgs/msg/Marker"))
        add_types.update(get_types_from_msg(_RIGID_BODY_MSG,   "mocap4r2_msgs/msg/RigidBody"))
        add_types.update(get_types_from_msg(_RIGID_BODIES_MSG, "mocap_msgs/msg/RigidBodies"))
    if include_jo_msgs:
        add_types.update(get_types_from_msg(_OBSTACLE_MSG,       "jo_msgs/msg/Obstacle"))
        add_types.update(get_types_from_msg(_OBSTACLE_ARRAY_MSG, "jo_msgs/msg/ObstacleArray"))
    if add_types:
        ts.register(add_types)
    return ts


def stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def make_transform(translation, quaternion) -> np.ndarray:
    t = np.array(translation, dtype=float)
    qx, qy, qz, qw = quaternion
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
    T[:3, 3] = t
    return T


def invert_transform(T: np.ndarray) -> np.ndarray:
    inv = np.eye(4)
    inv[:3, :3] = T[:3, :3].T
    inv[:3, 3] = -inv[:3, :3] @ T[:3, 3]
    return inv


def transform_point(T: np.ndarray, point) -> np.ndarray:
    p = np.array([point[0], point[1], point[2], 1.0])
    return (T @ p)[:3]


# ── Estrazione Qualisys ───────────────────────────────────────────────────────

def extract_qualisys_body(bag_path: str, topic: str, ros_version: str,
                          body_name: str) -> list:
    """
    Restituisce lista di (t_sec, x, y, z) per il rigid body richiesto.
    """
    ts = get_typestore_for_version(ros_version, include_mocap=True)
    positions = []

    with Reader(bag_path) as bag:
        connections = [c for c in bag.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in bag.connections})
            print(f"[ERROR] Topic '{topic}' non trovato in {bag_path}.")
            print("        Topic disponibili:\n  " + "\n  ".join(available))
            sys.exit(1)

        for conn, timestamp, rawdata in bag.messages(connections=connections):
            try:
                msg = ts.deserialize_cdr(rawdata, conn.msgtype)
            except Exception:
                continue
            try:
                t = stamp_to_sec(msg.header.stamp)
            except AttributeError:
                t = timestamp * 1e-9

            for rb in msg.rigidbodies:
                if rb.rigid_body_name == body_name:
                    p = rb.pose.position
                    if not (np.isfinite(p.x) and np.isfinite(p.y) and np.isfinite(p.z)):
                        break
                    positions.append((t, p.x, p.y, p.z))
                    break

    positions.sort(key=lambda r: r[0])
    print(f"[INFO] Qualisys '{body_name}': {len(positions)} campioni estratti da {Path(bag_path).name}")
    return positions


def extract_qualisys_body_poses(bag_path: str, topic: str, ros_version: str,
                                body_name: str) -> list:
    """Restituisce lista di (t_sec, T_qualisys_body)."""
    ts = get_typestore_for_version(ros_version, include_mocap=True)
    poses = []

    with Reader(bag_path) as bag:
        connections = [c for c in bag.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in bag.connections})
            print(f"[ERROR] Topic '{topic}' non trovato in {bag_path}.")
            print("        Topic disponibili:\n  " + "\n  ".join(available))
            sys.exit(1)

        for conn, timestamp, rawdata in bag.messages(connections=connections):
            try:
                msg = ts.deserialize_cdr(rawdata, conn.msgtype)
            except Exception:
                continue
            try:
                t = stamp_to_sec(msg.header.stamp)
            except AttributeError:
                t = timestamp * 1e-9

            for rb in msg.rigidbodies:
                if rb.rigid_body_name == body_name:
                    p = rb.pose.position
                    q = rb.pose.orientation
                    if not (np.isfinite(p.x) and np.isfinite(p.y) and np.isfinite(p.z)):
                        break
                    poses.append((t, make_transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))))
                    break

    poses.sort(key=lambda r: r[0])
    print(f"[INFO] Qualisys pose '{body_name}': {len(poses)} campioni estratti da {Path(bag_path).name}")
    return poses


def compute_velocity(positions: list, smooth_window: int = 9) -> list:
    """
    Calcola la velocità scalare in world frame con smoothing (moving average).
    Restituisce lista di (t_sec, |v|).
    """
    if len(positions) < 2:
        return []

    times = np.array([p[0] for p in positions])
    xyz   = np.array([[p[1], p[2], p[3]] for p in positions])

    dt   = np.diff(times)
    dxyz = np.diff(xyz, axis=0)
    speeds = np.linalg.norm(dxyz, axis=1) / np.maximum(dt, 1e-6)

    # Moving average smoothing
    w = max(1, smooth_window)
    kernel = np.ones(w) / w
    speeds_smooth = np.convolve(speeds, kernel, mode='same')

    # Associa ad ogni istante (escluso il primo) il valore smoothato
    result = [(times[i + 1], speeds_smooth[i]) for i in range(len(speeds_smooth))]
    return result


def interp_position(positions: list, t: float):
    """
    Interpola linearmente la posizione XYZ al tempo t.
    Restituisce (x, y, z) o None se fuori range.
    """
    if not positions:
        return None
    times = [p[0] for p in positions]
    if t < times[0] or t > times[-1]:
        return None
    idx = np.searchsorted(times, t)
    if idx == 0:
        return positions[0][1], positions[0][2], positions[0][3]
    if idx >= len(positions):
        return positions[-1][1], positions[-1][2], positions[-1][3]
    t0, x0, y0, z0 = positions[idx - 1]
    t1, x1, y1, z1 = positions[idx]
    alpha = (t - t0) / max(t1 - t0, 1e-9)
    return (x0 + alpha * (x1 - x0),
            y0 + alpha * (y1 - y0),
            z0 + alpha * (z1 - z0))


def nearest_pose(poses: list, t: float):
    if not poses:
        return None
    times = [p[0] for p in poses]
    if t < times[0] or t > times[-1]:
        return None
    idx = np.searchsorted(times, t)
    if idx == 0:
        return poses[0][1]
    if idx >= len(poses):
        return poses[-1][1]
    before = poses[idx - 1]
    after = poses[idx]
    return before[1] if abs(t - before[0]) <= abs(after[0] - t) else after[1]


def interp_speed(velocities: list, t: float) -> float:
    """Interpola la velocità scalare al tempo t."""
    if not velocities:
        return 0.0
    times = [v[0] for v in velocities]
    if t <= times[0]:
        return velocities[0][1]
    if t >= times[-1]:
        return velocities[-1][1]
    idx = np.searchsorted(times, t)
    t0, v0 = velocities[idx - 1]
    t1, v1 = velocities[idx]
    alpha = (t - t0) / max(t1 - t0, 1e-9)
    return v0 + alpha * (v1 - v0)


def transform_positions_xy(positions: list, yaw: float, tx: float, ty: float, tz: float) -> list:
    """
    Applica una trasformazione rigida planare alle posizioni Qualisys:
        p_odom = Rz(yaw) p_gt + [tx, ty, tz]
    """
    c = math.cos(yaw)
    s = math.sin(yaw)
    out = []
    for t, x, y, z in positions:
        xo = c * x - s * y + tx
        yo = s * x + c * y + ty
        zo = z + tz
        out.append((t, xo, yo, zo))
    return out


def select_alignment_obstacle(obstacles: list):
    """
    Sceglie l'ostacolo da usare per stimare l'allineamento.
    Negli esperimenti single-person si privilegiano le classificazioni
    semantiche/dinamiche rispetto alla sola track age, perché una track
    vecchia e stabile può essere clutter statico e falsare il frame GT->odom.
    """
    if not obstacles:
        return None

    def key(obs):
        status = obs.get('status', NOT_DETECTED)
        if status == STATUS_DYNAMIC:
            status_priority = 3
        elif status == STATUS_POTENTIALLY_DYNAMIC:
            status_priority = 2
        elif status == STATUS_TRACKED:
            status_priority = 1
        else:
            status_priority = 0
        return (status_priority, obs.get('track_age', 0))

    return max(obstacles, key=key)


def estimate_xy_translation(detector_frames: list,
                            gt_positions: list,
                            t_start_offset: float,
                            align_window: float,
                            gt_time_offset: float = 0.0) -> tuple:
    """
    Stima solo la traslazione XY tra GT e detector usando tempi relativi.
    Assunzione: nella finestra scelta l'ostacolo detector dominante è la persona.
    """
    if not detector_frames or not gt_positions:
        return 0.0, 0.0

    det_t0 = detector_frames[0][0]
    gt_t0 = gt_positions[0][0]
    det_xy = []
    gt_xy = []

    for t_det, obstacles in detector_frames:
        t_rel = t_det - det_t0
        if t_rel < t_start_offset:
            continue
        if t_rel > t_start_offset + align_window:
            break
        if not obstacles:
            continue

        obs = select_alignment_obstacle(obstacles)
        if obs is None:
            continue
        gt_pos = interp_position(gt_positions, gt_t0 + t_rel + gt_time_offset)
        if gt_pos is None:
            continue

        det_xy.append((obs['x'], obs['y']))
        gt_xy.append((gt_pos[0], gt_pos[1]))

    if len(det_xy) < 3:
        print("[WARN] Auto-align XY non affidabile: meno di 3 coppie detector/GT disponibili.")
        return 0.0, 0.0

    det_xy = np.array(det_xy)
    gt_xy = np.array(gt_xy)
    delta = np.median(det_xy - gt_xy, axis=0)
    return float(delta[0]), float(delta[1])


def collect_alignment_pairs(detector_frames: list,
                            gt_positions: list,
                            t_start_offset: float,
                            align_window: float,
                            gt_time_offset: float = 0.0) -> tuple:
    """
    Raccoglie coppie (GT xy, detector xy) su tempi relativi allineati.
    Per esperimenti single-person usa l'ostacolo più vecchio/stabile.
    """
    if not detector_frames or not gt_positions:
        return np.empty((0, 2)), np.empty((0, 2))

    det_t0 = detector_frames[0][0]
    gt_t0 = gt_positions[0][0]
    gt_xy = []
    det_xy = []

    for t_det, obstacles in detector_frames:
        t_rel = t_det - det_t0
        if t_rel < t_start_offset:
            continue
        if t_rel > t_start_offset + align_window:
            break
        if not obstacles:
            continue

        obs = select_alignment_obstacle(obstacles)
        if obs is None:
            continue
        gt_pos = interp_position(gt_positions, gt_t0 + t_rel + gt_time_offset)
        if gt_pos is None:
            continue

        gt_xy.append((gt_pos[0], gt_pos[1]))
        det_xy.append((obs['x'], obs['y']))

    return np.array(gt_xy), np.array(det_xy)


def estimate_se2_from_pairs(gt_xy: np.ndarray, det_xy: np.ndarray) -> tuple:
    """
    Stima yaw, tx, ty tali che det_xy ~= Rz(yaw) * gt_xy + t.
    """
    if len(gt_xy) < 3 or len(det_xy) < 3:
        return 0.0, 0.0, 0.0, float('inf')

    gt_centroid = np.mean(gt_xy, axis=0)
    det_centroid = np.mean(det_xy, axis=0)
    gt_centered = gt_xy - gt_centroid
    det_centered = det_xy - det_centroid

    h = gt_centered.T @ det_centered
    u, _, vt = np.linalg.svd(h)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0.0:
        vt[-1, :] *= -1.0
        r = vt.T @ u.T

    yaw = math.atan2(r[1, 0], r[0, 0])
    t = det_centroid - r @ gt_centroid
    pred = (r @ gt_xy.T).T + t
    residual = float(np.median(np.linalg.norm(pred - det_xy, axis=1)))
    return float(yaw), float(t[0]), float(t[1]), residual


def estimate_se2_alignment(detector_frames: list,
                           gt_positions: list,
                           t_start_offset: float,
                           align_window: float,
                           gt_time_offset: float = 0.0) -> tuple:
    gt_xy, det_xy = collect_alignment_pairs(
        detector_frames,
        gt_positions,
        t_start_offset=t_start_offset,
        align_window=align_window,
        gt_time_offset=gt_time_offset)
    yaw, tx, ty, residual = estimate_se2_from_pairs(gt_xy, det_xy)
    return yaw, tx, ty, residual, len(gt_xy)


def score_time_offset_alignment(detector_frames: list,
                                gt_positions: list,
                                t_start_offset: float,
                                align_window: float,
                                gt_time_offset: float,
                                use_se2: bool = False) -> tuple:
    """
    Valuta un offset temporale candidato stimando prima la sola traslazione XY
    e poi misurando la distanza residua detector-GT nella finestra scelta.
    """
    if not detector_frames or not gt_positions:
        return float('inf'), 0.0, 0.0, 0.0, 0

    if use_se2:
        yaw, dx, dy, residual, n_pairs = estimate_se2_alignment(
            detector_frames,
            gt_positions,
            t_start_offset=t_start_offset,
            align_window=align_window,
            gt_time_offset=gt_time_offset)
        return residual, yaw, dx, dy, n_pairs

    dx, dy = estimate_xy_translation(
        detector_frames,
        gt_positions,
        t_start_offset=t_start_offset,
        align_window=align_window,
        gt_time_offset=gt_time_offset)

    det_t0 = detector_frames[0][0]
    gt_t0 = gt_positions[0][0]
    residuals = []

    for t_det, obstacles in detector_frames:
        t_rel = t_det - det_t0
        if t_rel < t_start_offset:
            continue
        if t_rel > t_start_offset + align_window:
            break
        if not obstacles:
            continue

        obs = select_alignment_obstacle(obstacles)
        if obs is None:
            continue
        gt_pos = interp_position(gt_positions, gt_t0 + t_rel + gt_time_offset)
        if gt_pos is None:
            continue

        residuals.append(np.hypot(obs['x'] - (gt_pos[0] + dx),
                                  obs['y'] - (gt_pos[1] + dy)))

    if len(residuals) < 3:
        return float('inf'), 0.0, dx, dy, len(residuals)

    return float(np.median(residuals)), 0.0, dx, dy, len(residuals)


def estimate_time_offset_and_xy(detector_frames: list,
                                gt_positions: list,
                                t_start_offset: float,
                                align_window: float,
                                offset_min: float,
                                offset_max: float,
                                offset_step: float,
                                use_se2: bool = False) -> tuple:
    """
    Stima grossolanamente l'offset temporale GT rispetto al detector.
    Convenzione: t_gt = t_gt_start + t_relative + gt_time_offset.
    """
    step = max(abs(offset_step), 1e-3)
    if offset_max < offset_min:
        offset_min, offset_max = offset_max, offset_min

    best = (float('inf'), 0.0, 0.0, 0.0, 0.0, 0)
    n_steps = int(np.floor((offset_max - offset_min) / step)) + 1
    for k in range(n_steps):
        offset = offset_min + k * step
        residual, yaw, dx, dy, n_pairs = score_time_offset_alignment(
            detector_frames,
            gt_positions,
            t_start_offset=t_start_offset,
            align_window=align_window,
            gt_time_offset=offset,
            use_se2=use_se2)
        if residual < best[0]:
            best = (residual, offset, yaw, dx, dy, n_pairs)

    residual, offset, yaw, dx, dy, n_pairs = best
    if not np.isfinite(residual):
        print("[WARN] Auto-align time non affidabile: nessun offset con almeno 3 coppie detector/GT.")
        return 0.0, 0.0, 0.0, 0.0, 0

    return offset, yaw, dx, dy, n_pairs


def collect_track_alignment_pairs(detector_frames: list,
                                  gt_positions: list,
                                  track_id: int,
                                  t_start_offset: float,
                                  align_window: float,
                                  gt_time_offset: float = 0.0) -> tuple:
    if not detector_frames or not gt_positions:
        return np.empty((0, 2)), np.empty((0, 2))

    det_t0 = detector_frames[0][0]
    gt_t0 = gt_positions[0][0]
    gt_xy = []
    det_xy = []

    for t_det, obstacles in detector_frames:
        t_rel = t_det - det_t0
        if t_rel < t_start_offset:
            continue
        if t_rel > t_start_offset + align_window:
            break

        matches = [o for o in obstacles if o.get('track_id') == track_id]
        if not matches:
            continue

        gt_pos = interp_position(gt_positions, gt_t0 + t_rel + gt_time_offset)
        if gt_pos is None:
            continue

        obs = matches[0]
        gt_xy.append((gt_pos[0], gt_pos[1]))
        det_xy.append((obs['x'], obs['y']))

    return np.array(gt_xy), np.array(det_xy)


def estimate_track_time_offset_and_se2(detector_frames: list,
                                       gt_positions: list,
                                       track_id: int,
                                       t_start_offset: float,
                                       align_window: float,
                                       offset_min: float,
                                       offset_max: float,
                                       offset_step: float) -> tuple:
    step = max(abs(offset_step), 1e-3)
    if offset_max < offset_min:
        offset_min, offset_max = offset_max, offset_min

    best = (float('inf'), 0.0, 0.0, 0.0, 0.0, 0)
    n_steps = int(np.floor((offset_max - offset_min) / step)) + 1
    for k in range(n_steps):
        offset = offset_min + k * step
        gt_xy, det_xy = collect_track_alignment_pairs(
            detector_frames,
            gt_positions,
            track_id=track_id,
            t_start_offset=t_start_offset,
            align_window=align_window,
            gt_time_offset=offset)
        yaw, tx, ty, residual = estimate_se2_from_pairs(gt_xy, det_xy)
        if residual < best[0]:
            best = (residual, offset, yaw, tx, ty, len(gt_xy))

    residual, offset, yaw, tx, ty, n_pairs = best
    if not np.isfinite(residual):
        print(f"[WARN] Auto-align track_id={track_id} non affidabile: meno di 3 coppie detector/GT.")
        return 0.0, 0.0, 0.0, 0.0, 0

    return offset, yaw, tx, ty, n_pairs


def extract_odom_positions(bag_path: str, topic: str, ros_version: str) -> list:
    """Restituisce lista di (t_sec, x, y, z) dal topic nav_msgs/Odometry."""
    ts = get_typestore_for_version(ros_version)
    positions = []

    with Reader(bag_path) as bag:
        connections = [c for c in bag.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in bag.connections})
            print(f"[ERROR] Topic odom '{topic}' non trovato in {bag_path}.")
            print("        Topic disponibili:\n  " + "\n  ".join(available))
            sys.exit(1)

        for conn, timestamp, rawdata in bag.messages(connections=connections):
            try:
                msg = ts.deserialize_cdr(rawdata, conn.msgtype)
            except Exception as e:
                print(f"[WARN] Deserializzazione odom fallita: {e}")
                continue

            try:
                t = stamp_to_sec(msg.header.stamp)
            except AttributeError:
                t = timestamp * 1e-9

            p = msg.pose.pose.position
            if not (np.isfinite(p.x) and np.isfinite(p.y) and np.isfinite(p.z)):
                continue
            positions.append((t, p.x, p.y, p.z))

    positions.sort(key=lambda r: r[0])
    print(f"[INFO] Odom robot: {len(positions)} campioni estratti da {topic}")
    return positions


def extract_odom_poses(bag_path: str, topic: str, ros_version: str) -> list:
    """Restituisce lista di (t_sec, T_odom_base_link) dal topic nav_msgs/Odometry."""
    ts = get_typestore_for_version(ros_version)
    poses = []

    with Reader(bag_path) as bag:
        connections = [c for c in bag.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in bag.connections})
            print(f"[ERROR] Topic odom '{topic}' non trovato in {bag_path}.")
            print("        Topic disponibili:\n  " + "\n  ".join(available))
            sys.exit(1)

        for conn, timestamp, rawdata in bag.messages(connections=connections):
            try:
                msg = ts.deserialize_cdr(rawdata, conn.msgtype)
            except Exception as e:
                print(f"[WARN] Deserializzazione odom fallita: {e}")
                continue
            try:
                t = stamp_to_sec(msg.header.stamp)
            except AttributeError:
                t = timestamp * 1e-9

            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            poses.append((t, make_transform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))))

    poses.sort(key=lambda r: r[0])
    print(f"[INFO] Odom pose robot: {len(poses)} campioni estratti da {topic}")
    return poses


def extract_static_transform(bag_path: str, ros_version: str,
                             parent_frame: str, child_frame: str) -> np.ndarray:
    """Restituisce T_parent_child da /tf_static."""
    ts = get_typestore_for_version(ros_version)

    with Reader(bag_path) as bag:
        connections = [c for c in bag.connections if c.topic == '/tf_static']
        if not connections:
            print(f"[ERROR] Topic /tf_static non trovato in {bag_path}.")
            sys.exit(1)

        for conn, timestamp, rawdata in bag.messages(connections=connections):
            try:
                msg = ts.deserialize_cdr(rawdata, conn.msgtype)
            except Exception as e:
                print(f"[WARN] Deserializzazione tf_static fallita: {e}")
                continue
            for tf in msg.transforms:
                parent = tf.header.frame_id
                child = tf.child_frame_id
                tr = tf.transform.translation
                q = tf.transform.rotation
                T = make_transform((tr.x, tr.y, tr.z), (q.x, q.y, q.z, q.w))
                if parent == parent_frame and child == child_frame:
                    print(f"[INFO] Static TF trovata: {parent_frame} -> {child_frame}")
                    return T
                if parent == child_frame and child == parent_frame:
                    print(f"[INFO] Static TF trovata invertita: {child_frame} -> {parent_frame}")
                    return invert_transform(T)

    print(f"[ERROR] Static TF {parent_frame} -> {child_frame} non trovata in /tf_static.")
    sys.exit(1)


def collect_robot_alignment_pairs(odom_positions: list,
                                  robot_gt_positions: list,
                                  detector_start_time: float,
                                  t_start_offset: float,
                                  align_window: float,
                                  gt_time_offset: float = 0.0) -> tuple:
    if not odom_positions or not robot_gt_positions:
        return np.empty((0, 2)), np.empty((0, 2))

    # Odometry may keep the original bag timestamps while detector outputs may
    # be stamped at replay/evaluation time. For robot-based frame alignment we
    # compare JO and odom on the odometry clock itself, then reuse the estimated
    # spatial transform for the detector-vs-person evaluation.
    odom_t0 = odom_positions[0][0]
    gt_t0 = robot_gt_positions[0][0]
    gt_xy = []
    odom_xy = []

    for t_odom, x_odom, y_odom, _ in odom_positions:
        t_rel = t_odom - odom_t0
        if t_rel < t_start_offset:
            continue
        if t_rel > t_start_offset + align_window:
            break

        gt_pos = interp_position(robot_gt_positions, gt_t0 + t_rel + gt_time_offset)
        if gt_pos is None:
            continue

        gt_xy.append((gt_pos[0], gt_pos[1]))
        odom_xy.append((x_odom, y_odom))

    return np.array(gt_xy), np.array(odom_xy)


def estimate_robot_time_offset_and_se2(odom_positions: list,
                                       robot_gt_positions: list,
                                       detector_start_time: float,
                                       t_start_offset: float,
                                       align_window: float,
                                       offset_min: float,
                                       offset_max: float,
                                       offset_step: float) -> tuple:
    step = max(abs(offset_step), 1e-3)
    if offset_max < offset_min:
        offset_min, offset_max = offset_max, offset_min

    best = (float('inf'), 0.0, 0.0, 0.0, 0.0, 0)
    n_steps = int(np.floor((offset_max - offset_min) / step)) + 1
    for k in range(n_steps):
        offset = offset_min + k * step
        gt_xy, odom_xy = collect_robot_alignment_pairs(
            odom_positions,
            robot_gt_positions,
            detector_start_time=detector_start_time,
            t_start_offset=t_start_offset,
            align_window=align_window,
            gt_time_offset=offset)
        yaw, tx, ty, residual = estimate_se2_from_pairs(gt_xy, odom_xy)
        if residual < best[0]:
            best = (residual, offset, yaw, tx, ty, len(gt_xy))

    residual, offset, yaw, tx, ty, n_pairs = best
    if not np.isfinite(residual):
        print("[WARN] Robot-based auto-align non affidabile: meno di 3 coppie JO/odom.")
        return 0.0, 0.0, 0.0, 0.0, 0

    return offset, yaw, tx, ty, n_pairs


# ── Estrazione detector output ────────────────────────────────────────────────

def extract_detector_frames(bag_path: str, topic: str, ros_version: str) -> list:
    """
    Restituisce lista di (t_sec, [dict obstacle]).
    Ogni obstacle: {'x', 'y', 'z', 'status', 'track_id', 'track_age'}
    """
    ts = get_typestore_for_version(ros_version, include_jo_msgs=True)
    frames = []

    with Reader(bag_path) as bag:
        connections = [c for c in bag.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in bag.connections})
            print(f"[ERROR] Topic '{topic}' non trovato in {bag_path}.")
            print("        Topic disponibili:\n  " + "\n  ".join(available))
            sys.exit(1)

        for conn, timestamp, rawdata in bag.messages(connections=connections):
            try:
                msg = ts.deserialize_cdr(rawdata, conn.msgtype)
            except Exception as e:
                print(f"[WARN] Deserializzazione fallita: {e}")
                continue

            try:
                t = stamp_to_sec(msg.header.stamp)
            except AttributeError:
                t = timestamp * 1e-9

            obstacles = []
            for obs in msg.obstacles:
                p = obs.pose.position
                obstacles.append({
                    'x':         p.x,
                    'y':         p.y,
                    'z':         p.z,
                    'vx':        obs.twist.linear.x,
                    'vy':        obs.twist.linear.y,
                    'status':    int(obs.status),
                    'track_id':  int(obs.track_id),
                    'track_age': int(obs.track_age),
                })
            frames.append((t, obstacles))

    frames.sort(key=lambda r: r[0])
    print(f"[INFO] Detector: {len(frames)} frame estratti da {Path(bag_path).name}")
    return frames


def transform_detector_frames_odom_to_qualisys(detector_frames: list,
                                               odom_poses: list,
                                               robot_gt_poses: list,
                                               T_base_imu: np.ndarray,
                                               gt_time_offset: float,
                                               t_start_offset: float,
                                               odom_time_offset: float = 0.0) -> list:
    """
    Trasforma ogni obstacle center da odom a Qualisys:
      p_odom -> base_link usando T_odom_base da /odometry/filtered
      base_link -> imu_link usando TF static T_base_imu
      imu_link -> Qualisys usando la posa Qualisys del rigid body JO≈imu_link
    """
    if not detector_frames or not odom_poses or not robot_gt_poses:
        return detector_frames

    det_t0 = detector_frames[0][0]
    odom_t0 = odom_poses[0][0]
    gt_t0 = robot_gt_poses[0][0]
    T_imu_base = invert_transform(T_base_imu)
    out = []
    missing = 0

    for t_det, obstacles in detector_frames:
        t_rel = t_det - det_t0
        t_odom = odom_t0 + t_rel + odom_time_offset
        t_gt = gt_t0 + t_rel + gt_time_offset
        T_odom_base = nearest_pose(odom_poses, t_odom)
        T_qual_imu = nearest_pose(robot_gt_poses, t_gt)
        if T_odom_base is None or T_qual_imu is None:
            out.append((t_det, []))
            missing += 1
            continue

        T_base_odom = invert_transform(T_odom_base)
        transformed = []
        for obs in obstacles:
            p_base = transform_point(T_base_odom, (obs['x'], obs['y'], obs['z']))
            p_imu = transform_point(T_imu_base, p_base)
            p_qual = transform_point(T_qual_imu, p_imu)
            obs_q = dict(obs)
            obs_q['x'] = float(p_qual[0])
            obs_q['y'] = float(p_qual[1])
            obs_q['z'] = float(p_qual[2])
            transformed.append(obs_q)
        out.append((t_det, transformed))

    if missing:
        print(f"[WARN] {missing} detector frame senza posa odom/JO valida per trasformazione in Qualisys.")
    print("[INFO] Detector obstacles transformed: odom -> base_link -> imu_link -> Qualisys")
    return out


# ── Valutazione ───────────────────────────────────────────────────────────────

def evaluate(detector_frames: list,
             person_positions: list,
             person_velocities: list,
             t_start_offset: float,
             match_dist_thresh: float,
             dyn_vel_thresh: float,
             static_vel_thresh: float,
             force_no_gt_potentially_dynamic: bool,
             t_max_diff: float,
             time_mode: str,
             gt_time_offset: float) -> dict:
    """
    Valuta frame per frame.

    Ritorna dizionario con:
        records: lista di dict per ogni frame valutato
        counters: {DYNAMIC/POTENTIALLY_DYNAMIC/STATIC: {tp,fp,fn,tn}}
        detection_total, detection_found
    """
    if not detector_frames or not person_positions:
        return {}

    t_det_start = detector_frames[0][0]
    t_gt_start  = person_positions[0][0]
    t_abs_start = max(t_det_start, t_gt_start) + t_start_offset

    counters = {cls: {'tp': 0, 'fp': 0, 'fn': 0, 'tn': 0} for cls in GT_CLASSES}
    confusion = {
        gt_cls: {pred_cls: 0 for pred_cls in PRED_CLASSES}
        for gt_cls in GT_CLASSES
    }
    records = []
    detection_total = 0
    detection_found = 0
    classification_total = 0

    def add_sample(gt_label, pred_label, record):
        nonlocal classification_total
        if pred_label not in PRED_CLASSES:
            pred_label = NOT_DETECTED
            record['pred_label'] = pred_label
        confusion[gt_label][pred_label] += 1
        classification_total += 1

        # One-vs-rest counters. The multiclass confusion matrix remains the
        # primary output, while these preserve per-class precision/recall.
        for cls in GT_CLASSES:
            gt_pos_cls   = (gt_label == cls)
            pred_pos_cls = (pred_label == cls)
            if gt_pos_cls and pred_pos_cls:
                counters[cls]['tp'] += 1
            elif not gt_pos_cls and pred_pos_cls:
                counters[cls]['fp'] += 1
            elif gt_pos_cls and not pred_pos_cls:
                counters[cls]['fn'] += 1
            else:
                counters[cls]['tn'] += 1

        records.append(record)

    for t_det, obstacles in detector_frames:
        if time_mode == "relative":
            t_rel = t_det - t_det_start
            if t_rel < t_start_offset:
                continue
            t_gt = t_gt_start + t_rel + gt_time_offset
        else:
            if t_det < t_abs_start:
                continue
            t_rel = t_det - t_det_start
            t_gt = t_det + gt_time_offset

        # Posizione GT persona temporalmente allineata al frame detector.
        gt_pos = interp_position(person_positions, t_gt)
        if gt_pos is None:
            continue  # fuori dal range Qualisys

        gt_speed = interp_speed(person_velocities, t_gt)
        if gt_speed > dyn_vel_thresh:
            gt_label = STATUS_DYNAMIC
        elif force_no_gt_potentially_dynamic:
            gt_label = STATUS_DYNAMIC
        elif gt_speed <= static_vel_thresh:
            gt_label = STATUS_STATIC
        else:
            gt_label = STATUS_POTENTIALLY_DYNAMIC

        # Trova l'ostacolo più vicino alla persona. Solo questa box può essere
        # associata al target dinamico/PD; le altre predizioni del frame sono
        # trattate come GT STATIC, cioè come detections su background/oggetti
        # non corrispondenti alla persona target.
        best_idx = -1
        best_obs = None
        best_dist = float('inf')
        obstacle_distances = []
        for idx, obs in enumerate(obstacles):
            d = np.sqrt((obs['x'] - gt_pos[0])**2 +
                        (obs['y'] - gt_pos[1])**2)
            obstacle_distances.append(d)
            if d < best_dist:
                best_dist = d
                best_obs  = obs
                best_idx = idx

        detection_total += 1
        matched_idx = -1
        if best_obs is not None and best_dist <= match_dist_thresh:
            pred_label = best_obs['status']
            # TRACKED is not a classifier output; collapse it into STATIC
            if pred_label == STATUS_TRACKED:
                pred_label = STATUS_STATIC
            detection_found += 1
            matched_idx = best_idx
        else:
            pred_label = NOT_DETECTED

        add_sample(gt_label, pred_label, {
            'sample_type':    'target',
            't':            t_det,
            't_rel':        t_rel,
            't_gt':         t_gt,
            'gt_label':     gt_label,
            'pred_label':   pred_label,
            'gt_speed':     gt_speed,
            'match_dist':   best_dist if best_obs is not None else float('nan'),
            'gt_x':         gt_pos[0],
            'gt_y':         gt_pos[1],
            'pred_x':       best_obs['x'] if best_obs is not None else float('nan'),
            'pred_y':       best_obs['y'] if best_obs is not None else float('nan'),
            'pred_vx':      best_obs['vx'] if best_obs is not None else float('nan'),
            'pred_vy':      best_obs['vy'] if best_obs is not None else float('nan'),
            'track_id':     best_obs['track_id'] if best_obs is not None else -1,
            'track_age':    best_obs['track_age'] if best_obs is not None else -1,
        })

        for idx, obs in enumerate(obstacles):
            if idx == matched_idx:
                continue

            static_pred_label = obs['status']
            if static_pred_label == STATUS_TRACKED:
                static_pred_label = STATUS_STATIC
            if static_pred_label not in PRED_CLASSES:
                static_pred_label = NOT_DETECTED

            add_sample(STATUS_STATIC, static_pred_label, {
                'sample_type':    'unmatched_prediction',
                't':              t_det,
                't_rel':          t_rel,
                't_gt':           t_gt,
                'gt_label':       STATUS_STATIC,
                'pred_label':     static_pred_label,
                'gt_speed':       float('nan'),
                'match_dist':     obstacle_distances[idx] if idx < len(obstacle_distances) else float('nan'),
                'gt_x':           float('nan'),
                'gt_y':           float('nan'),
                'pred_x':         obs['x'],
                'pred_y':         obs['y'],
                'pred_vx':        obs['vx'],
                'pred_vy':        obs['vy'],
                'track_id':       obs['track_id'],
                'track_age':      obs['track_age'],
            })

    return {
        'records':               records,
        'counters':              counters,
        'confusion':             confusion,
        'detection_total':       detection_total,
        'detection_found':       detection_found,
        'classification_total':  classification_total,
    }


def compute_metrics(c: dict) -> dict:
    tp, fp, fn, tn = c['tp'], c['fp'], c['fn'], c['tn']
    total = tp + fp + fn + tn
    support = tp + fn
    predicted = tp + fp
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1 = (2 * precision * recall / (precision + recall)
          if (precision + recall) > 0 else 0.0)
    accuracy  = (tp + tn) / total if total > 0 else float('nan')
    specificity = tn / (tn + fp) if (tn + fp) > 0 else 0.0
    balanced_accuracy = (
        (recall + specificity) / 2 if total > 0 else float('nan')
    )
    return {'precision': precision, 'recall': recall, 'f1': f1, 'accuracy': accuracy,
            'specificity': specificity, 'balanced_accuracy': balanced_accuracy,
            'support': support, 'predicted': predicted,
            'tp': tp, 'fp': fp, 'fn': fn, 'tn': tn}


def summarize_results(results: dict) -> dict:
    per_class = {cls: compute_metrics(results['counters'][cls]) for cls in GT_CLASSES}

    correct = sum(results['confusion'][cls][cls] for cls in GT_CLASSES)
    total = results['classification_total']
    detection_rate = (
        results['detection_found'] / results['detection_total']
        if results['detection_total'] > 0 else float('nan')
    )

    return {
        'per_class': per_class,
        'detection_rate': detection_rate,
        'micro_accuracy': correct / total if total > 0 else float('nan'),
        'correct': correct,
        'classification_total': total,
        # Unweighted macro metrics: every GT label has the same weight,
        # independently from its support in the dataset.
        'macro_precision': sum(per_class[cls]['precision'] for cls in GT_CLASSES) / len(GT_CLASSES),
        'macro_recall': sum(per_class[cls]['recall'] for cls in GT_CLASSES) / len(GT_CLASSES),
        'macro_f1': sum(per_class[cls]['f1'] for cls in GT_CLASSES) / len(GT_CLASSES),
        'balanced_accuracy': sum(per_class[cls]['recall'] for cls in GT_CLASSES) / len(GT_CLASSES),
        'multiclass_mcc': compute_multiclass_mcc(results['confusion']),
    }


def compute_multiclass_mcc(confusion: dict) -> float:
    """Multiclass Matthews correlation coefficient.

    The confusion matrix includes NOT_DETECTED as a prediction-only label with
    zero GT support, so missed target detections are penalized explicitly.
    """
    labels = (STATUS_DYNAMIC, STATUS_POTENTIALLY_DYNAMIC, STATUS_STATIC, NOT_DETECTED)
    matrix = np.zeros((len(labels), len(labels)), dtype=float)
    label_to_idx = {label: idx for idx, label in enumerate(labels)}

    for gt_label in GT_CLASSES:
        row = confusion[gt_label]
        for pred_label in labels:
            matrix[label_to_idx[gt_label], label_to_idx[pred_label]] = row.get(pred_label, 0)

    samples = matrix.sum()
    if samples <= 0:
        return float('nan')

    trace = np.trace(matrix)
    pred_totals = matrix.sum(axis=0)
    true_totals = matrix.sum(axis=1)
    cov_ytyp = trace * samples - np.dot(pred_totals, true_totals)
    cov_ypyp = samples ** 2 - np.dot(pred_totals, pred_totals)
    cov_ytyt = samples ** 2 - np.dot(true_totals, true_totals)
    denom = math.sqrt(cov_ypyp * cov_ytyt)
    return cov_ytyp / denom if denom > 0 else 0.0


# ── Output ────────────────────────────────────────────────────────────────────

def print_results(exp_name: str, results: dict, dyn_vel_thresh: float):
    summary = summarize_results(results)
    det_rate = summary['detection_rate']

    print(f"\n{'═'*60}")
    print(f"  Esperimento: {exp_name}")
    print(f"  Frame valutati: {results['detection_total']}  "
          f"| Persona rilevata: {results['detection_found']} "
          f"({det_rate*100:.1f}%)")
    print(f"  GT vel thresh: {dyn_vel_thresh} m/s")
    print(f"{'─'*60}")
    print("  [MULTICLASS CONFUSION MATRIX]")
    print("    GT \\ Pred             DYNAMIC  POT_DYN   STATIC  NOT_DET")
    for gt_cls in GT_CLASSES:
        row = results['confusion'][gt_cls]
        gt_name = LABEL_MAP[gt_cls]
        print(f"    {gt_name:22s}"
              f"{row[STATUS_DYNAMIC]:8d}"
              f"{row[STATUS_POTENTIALLY_DYNAMIC]:9d}"
              f"{row[STATUS_STATIC]:9d}"
              f"{row[NOT_DETECTED]:9d}")
    print(f"    Accuracy multiclass: {summary['correct']}/{summary['classification_total']} "
          f"({summary['micro_accuracy']:.3f})")
    print(f"    Macro Precision={summary['macro_precision']:.3f}  "
          f"Macro Recall={summary['macro_recall']:.3f}  "
          f"Macro F1={summary['macro_f1']:.3f}  "
          f"MCC={summary['multiclass_mcc']:.3f}")
    print(f"{'─'*60}")

    for cls, label in ((STATUS_DYNAMIC,             'DYNAMIC'),
                       (STATUS_POTENTIALLY_DYNAMIC, 'POTENTIALLY_DYNAMIC'),
                       (STATUS_STATIC,              'STATIC')):
        m = summary['per_class'][cls]
        print(f"  [{label}]")
        print(f"    TP={m['tp']:4d}  FP={m['fp']:4d}  FN={m['fn']:4d}  TN={m['tn']:4d}  "
              f"Support={m['support']:4d}")
        print(f"    Precision={m['precision']:.3f}  Recall={m['recall']:.3f}  "
              f"F1={m['f1']:.3f}  Accuracy={m['accuracy']:.3f}")

    print(f"{'═'*60}")


def save_csv(out_dir: str, exp_name: str, results: dict, dyn_vel_thresh: float):
    os.makedirs(out_dir, exist_ok=True)
    summary = summarize_results(results)
    det_rate = summary['detection_rate']

    summary_path = os.path.join(out_dir, f"{exp_name}_metrics.csv")
    with open(summary_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['experiment', 'class', 'tp', 'fp', 'fn', 'tn',
                    'support', 'predicted',
                    'precision', 'recall', 'f1', 'accuracy',
                    'specificity', 'balanced_accuracy',
                    'detection_rate', 'frames_evaluated',
                    'classification_samples'])
        for cls, label in ((STATUS_DYNAMIC,             'DYNAMIC'),
                           (STATUS_POTENTIALLY_DYNAMIC, 'POTENTIALLY_DYNAMIC'),
                           (STATUS_STATIC,              'STATIC')):
            m = summary['per_class'][cls]
            w.writerow([exp_name, label,
                        m['tp'], m['fp'], m['fn'], m['tn'],
                        m['support'], m['predicted'],
                        f"{m['precision']:.4f}", f"{m['recall']:.4f}",
                        f"{m['f1']:.4f}", f"{m['accuracy']:.4f}",
                        f"{m['specificity']:.4f}", f"{m['balanced_accuracy']:.4f}",
                        f"{det_rate:.4f}", results['detection_total'],
                        results['classification_total']])
        w.writerow([exp_name, 'MACRO_AVERAGE', '', '', '', '', '', '',
                    f"{summary['macro_precision']:.4f}",
                    f"{summary['macro_recall']:.4f}",
                    f"{summary['macro_f1']:.4f}",
                    f"{summary['micro_accuracy']:.4f}",
                    '',
                    f"{summary['balanced_accuracy']:.4f}",
                    f"{det_rate:.4f}", results['detection_total'],
                    results['classification_total']])
        w.writerow([exp_name, 'MICRO_ACCURACY', '', '', '', '', '', '',
                    '', '', '', f"{summary['micro_accuracy']:.4f}", '',
                    '', f"{det_rate:.4f}", results['detection_total'],
                    results['classification_total']])
        w.writerow([exp_name, 'MULTICLASS_MCC', '', '', '', '', '', '',
                    '', '', f"{summary['multiclass_mcc']:.4f}", '', '',
                    '', f"{det_rate:.4f}", results['detection_total'],
                    results['classification_total']])

    confusion_path = os.path.join(out_dir, f"{exp_name}_confusion_matrix.csv")
    with open(confusion_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['experiment', 'gt_label',
                    'pred_dynamic', 'pred_potentially_dynamic',
                    'pred_static', 'pred_not_detected',
                    'gt_total', 'correct', 'class_recall'])
        for gt_cls in GT_CLASSES:
            row = results['confusion'][gt_cls]
            gt_total = sum(row.values())
            correct = row[gt_cls]
            recall = correct / gt_total if gt_total > 0 else float('nan')
            w.writerow([exp_name, LABEL_MAP[gt_cls],
                        row[STATUS_DYNAMIC],
                        row[STATUS_POTENTIALLY_DYNAMIC],
                        row[STATUS_STATIC],
                        row[NOT_DETECTED],
                        gt_total,
                        correct,
                        f"{recall:.4f}" if not np.isnan(recall) else 'nan'])

        total = results['classification_total']
        correct = sum(results['confusion'][cls][cls] for cls in GT_CLASSES)
        w.writerow([exp_name, 'MULTICLASS_TOTAL', '', '', '', '',
                    total, correct,
                    f"{correct / total:.4f}" if total > 0 else 'nan'])

    records_path = os.path.join(out_dir, f"{exp_name}_frames.csv")
    with open(records_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['sample_type', 't_detector', 't_relative', 't_gt',
                    'gt_label', 'pred_label', 'gt_speed_m_s', 'match_dist_m',
                    'gt_x_odom', 'gt_y_odom', 'pred_x_odom', 'pred_y_odom',
                    'pred_vx', 'pred_vy', 'track_id', 'track_age'])
        for r in results['records']:
            w.writerow([r['sample_type'],
                        f"{r['t']:.6f}",
                        f"{r['t_rel']:.6f}",
                        f"{r['t_gt']:.6f}",
                        LABEL_MAP.get(r['gt_label'],   str(r['gt_label'])),
                        LABEL_MAP.get(r['pred_label'], str(r['pred_label'])),
                        f"{r['gt_speed']:.4f}" if not np.isnan(r['gt_speed']) else 'nan',
                        f"{r['match_dist']:.3f}" if not np.isnan(r['match_dist']) else 'nan',
                        f"{r['gt_x']:.3f}" if not np.isnan(r['gt_x']) else 'nan',
                        f"{r['gt_y']:.3f}" if not np.isnan(r['gt_y']) else 'nan',
                        f"{r['pred_x']:.3f}" if not np.isnan(r['pred_x']) else 'nan',
                        f"{r['pred_y']:.3f}" if not np.isnan(r['pred_y']) else 'nan',
                        f"{r['pred_vx']:.3f}" if not np.isnan(r['pred_vx']) else 'nan',
                        f"{r['pred_vy']:.3f}" if not np.isnan(r['pred_vy']) else 'nan',
                        r['track_id'],
                        r['track_age']])

    print(f"[INFO] Salvato → {summary_path}")
    print(f"[INFO] Salvato → {confusion_path}")
    print(f"[INFO] Salvato → {records_path}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Valuta classificazione Dynamic/PotentiallyDynamic di jo_detector")
    parser.add_argument("--detector_bag",          required=True,
                        help="Bag ROS2 con output del detector (.mcap o .db3)")
    parser.add_argument("--gt_bag",                required=True,
                        help="Bag Qualisys con posizioni rigide (.db3 o .mcap)")
    parser.add_argument("--gt_topic",              default="/rigid_bodies",
                        help="Topic Qualisys RigidBodies (default: /rigid_bodies)")
    parser.add_argument("--person_body",           required=True,
                        help="Nome rigid body della persona in Qualisys")
    parser.add_argument("--robot_body",            default="",
                        help="Nome rigid body del robot in Qualisys, es. JO")
    parser.add_argument("--odom_topic",            default="/odometry/filtered",
                        help="Topic nav_msgs/Odometry del robot nel frame odom")
    parser.add_argument("--align_with_robot", action="store_true",
                        help="Stima GT->odom usando robot_body Qualisys e odom_topic")
    parser.add_argument("--compare_in_qualisys", action="store_true",
                        help="Porta le box detector odom->base_link->imu_link->Qualisys e confronta nel frame Qualisys")
    parser.add_argument("--base_frame", default="base_link",
                        help="Frame base usato da /odometry/filtered")
    parser.add_argument("--imu_frame", default="imu_link",
                        help="Frame Qualisys JO coincidente con questo frame robot")
    parser.add_argument("--odom_time_offset", type=float, default=0.0,
                        help="Offset temporale applicato a /odometry/filtered nella modalità compare_in_qualisys")
    parser.add_argument("--detector_topic",
                        default="/perception/tracked_dynamic_obstacles",
                        help="Topic ObstacleArray del detector")
    parser.add_argument("--detector_ros_version",  default="jazzy")
    parser.add_argument("--gt_ros_version",        default="humble")
    parser.add_argument("--output_dir",            default="./results")
    parser.add_argument("--exp_name",              default=None,
                        help="Nome esperimento (default: stem del bag del detector)")
    parser.add_argument("--t_start",       type=float, default=10.0,
                        help="Secondi da saltare all'inizio (default: 10)")
    parser.add_argument("--dyn_vel_thresh", type=float, default=0.15,
                        help="Soglia velocità [m/s] per GT=DYNAMIC (default: 0.15)")
    parser.add_argument("--static_vel_thresh", type=float, default=-1.0,
                        help=("Soglia velocità [m/s] sotto cui il GT diventa STATIC. "
                              "Default -1.0: disabilitata, quindi i non-dynamic restano "
                              "POTENTIALLY_DYNAMIC."))
    parser.add_argument("--force_no_gt_potentially_dynamic", action="store_true",
                        help=("Forza a DYNAMIC tutti i frame GT non STATIC che non superano "
                              "dyn_vel_thresh. Utile per run in cui la persona e' nota "
                              "come sempre dinamica e i residui POT_DYN sono artefatti."))
    parser.add_argument("--match_dist_thresh", type=float, default=1.0,
                        help="Distanza max XY [m] per associare ostacolo a persona (default: 1.0)")
    parser.add_argument("--t_max_diff",    type=float, default=0.1,
                        help="Tolleranza tempo nominale [s] (mantenuta per compatibilità)")
    parser.add_argument("--vel_smooth_window", type=int, default=9,
                        help="Finestra moving-average velocità Qualisys (default: 9 campioni)")
    parser.add_argument("--time_mode", choices=["relative", "absolute"], default="relative",
                        help="relative: allinea le bag rispetto al rispettivo start; absolute: usa timestamp assoluti")
    parser.add_argument("--gt_time_offset", type=float, default=0.0,
                        help="Offset aggiunto al tempo GT dopo l'allineamento [s]")
    parser.add_argument("--gt_yaw", type=float, default=0.0,
                        help="Yaw [rad] della trasformazione GT->odom")
    parser.add_argument("--gt_tx", type=float, default=0.0,
                        help="Traslazione X [m] della trasformazione GT->odom")
    parser.add_argument("--gt_ty", type=float, default=0.0,
                        help="Traslazione Y [m] della trasformazione GT->odom")
    parser.add_argument("--gt_tz", type=float, default=0.0,
                        help="Traslazione Z [m] della trasformazione GT->odom")
    parser.add_argument("--auto_align_xy", action="store_true",
                        help="Stima una traslazione XY GT->odom dalla mediana detector-GT")
    parser.add_argument("--auto_align_se2", action="store_true",
                        help="Stima yaw+traslazione XY GT->odom dalla traiettoria detector-GT")
    parser.add_argument("--auto_align_track_id", type=int, default=-1,
                        help="Se >=0, stima time+SE2 usando solo questa track del detector come target")
    parser.add_argument("--auto_align_window", type=float, default=10.0,
                        help="Finestra [s] usata per gli auto-align dopo t_start")
    parser.add_argument("--auto_align_time", action="store_true",
                        help="Stima gt_time_offset cercando l'offset che minimizza la distanza detector-GT")
    parser.add_argument("--auto_align_time_min", type=float, default=-5.0,
                        help="Offset temporale minimo [s] per --auto_align_time")
    parser.add_argument("--auto_align_time_max", type=float, default=5.0,
                        help="Offset temporale massimo [s] per --auto_align_time")
    parser.add_argument("--auto_align_time_step", type=float, default=0.1,
                        help="Passo di ricerca [s] per --auto_align_time")
    args = parser.parse_args()

    exp_name = args.exp_name or Path(args.detector_bag).stem

    # Estrai posizioni persona da Qualisys
    person_positions = extract_qualisys_body(
        args.gt_bag, args.gt_topic, args.gt_ros_version, args.person_body)
    if not person_positions:
        print(f"[ERROR] Nessuna posizione trovata per il rigid body '{args.person_body}'.")
        sys.exit(1)

    # Calcola velocità persona
    person_velocities = compute_velocity(person_positions, args.vel_smooth_window)

    # Estrai frame del detector
    detector_frames = extract_detector_frames(
        args.detector_bag, args.detector_topic, args.detector_ros_version)
    if not detector_frames:
        print("[ERROR] Nessun frame estratto dal detector.")
        sys.exit(1)

    print("[INFO] Time alignment:")
    print(f"       detector range: {detector_frames[0][0]:.6f} -> {detector_frames[-1][0]:.6f}")
    print(f"       GT range:       {person_positions[0][0]:.6f} -> {person_positions[-1][0]:.6f}")
    print(f"       mode={args.time_mode}, gt_time_offset={args.gt_time_offset:.3f}s")

    effective_gt_time_offset = args.gt_time_offset

    if args.compare_in_qualisys:
        if not args.robot_body:
            print("[ERROR] --compare_in_qualisys richiede --robot_body, es. --robot_body JO")
            sys.exit(1)
        robot_poses = extract_qualisys_body_poses(
            args.gt_bag, args.gt_topic, args.gt_ros_version, args.robot_body)
        odom_poses = extract_odom_poses(
            args.detector_bag, args.odom_topic, args.detector_ros_version)
        T_base_imu = extract_static_transform(
            args.detector_bag, args.detector_ros_version, args.base_frame, args.imu_frame)
        detector_frames = transform_detector_frames_odom_to_qualisys(
            detector_frames,
            odom_poses,
            robot_poses,
            T_base_imu,
            gt_time_offset=effective_gt_time_offset,
            t_start_offset=args.t_start,
            odom_time_offset=args.odom_time_offset)
        print(f"[INFO] Evaluation frame: Qualisys. Chain: odom -> {args.base_frame} -> {args.imu_frame} -> Qualisys/{args.robot_body}")
    else:
        person_positions = transform_positions_xy(
            person_positions,
            yaw=args.gt_yaw,
            tx=args.gt_tx,
            ty=args.gt_ty,
            tz=args.gt_tz)

    if (not args.compare_in_qualisys) and args.align_with_robot:
        if not args.robot_body:
            print("[ERROR] --align_with_robot richiede --robot_body, es. --robot_body JO")
            sys.exit(1)
        robot_positions = extract_qualisys_body(
            args.gt_bag, args.gt_topic, args.gt_ros_version, args.robot_body)
        if not robot_positions:
            print(f"[ERROR] Nessuna posizione Qualisys trovata per robot_body '{args.robot_body}'.")
            sys.exit(1)
        robot_positions = transform_positions_xy(
            robot_positions,
            yaw=args.gt_yaw,
            tx=args.gt_tx,
            ty=args.gt_ty,
            tz=args.gt_tz)
        odom_positions = extract_odom_positions(
            args.detector_bag, args.odom_topic, args.detector_ros_version)
        if not odom_positions:
            print("[ERROR] Nessuna posizione odometrica estratta.")
            sys.exit(1)
        offset, yaw, dx, dy, n_pairs = estimate_robot_time_offset_and_se2(
            odom_positions,
            robot_positions,
            detector_start_time=detector_frames[0][0],
            t_start_offset=args.t_start,
            align_window=args.auto_align_window,
            offset_min=args.auto_align_time_min,
            offset_max=args.auto_align_time_max,
            offset_step=args.auto_align_time_step)
        effective_gt_time_offset = offset
        person_positions = transform_positions_xy(person_positions, yaw, dx, dy, 0.0)
        print(f"[INFO] Robot-based GT->odom applied using '{args.robot_body}' and '{args.odom_topic}': "
              f"gt_time_offset={offset:.3f}s, yaw={yaw:.3f} rad, "
              f"dx={dx:.3f} m, dy={dy:.3f} m, pairs={n_pairs}")
    elif args.auto_align_track_id >= 0:
        offset, yaw, dx, dy, n_pairs = estimate_track_time_offset_and_se2(
            detector_frames,
            person_positions,
            track_id=args.auto_align_track_id,
            t_start_offset=args.t_start,
            align_window=args.auto_align_window,
            offset_min=args.auto_align_time_min,
            offset_max=args.auto_align_time_max,
            offset_step=args.auto_align_time_step)
        effective_gt_time_offset = offset
        person_positions = transform_positions_xy(person_positions, yaw, dx, dy, 0.0)
        print(f"[INFO] Auto-align track_id={args.auto_align_track_id} applied: "
              f"gt_time_offset={offset:.3f}s, yaw={yaw:.3f} rad, "
              f"dx={dx:.3f} m, dy={dy:.3f} m, pairs={n_pairs}")
    elif args.auto_align_time:
        offset, yaw, dx, dy, n_pairs = estimate_time_offset_and_xy(
            detector_frames,
            person_positions,
            t_start_offset=args.t_start,
            align_window=args.auto_align_window,
            offset_min=args.auto_align_time_min,
            offset_max=args.auto_align_time_max,
            offset_step=args.auto_align_time_step,
            use_se2=args.auto_align_se2)
        effective_gt_time_offset = offset
        person_positions = transform_positions_xy(person_positions, yaw, dx, dy, 0.0)
        mode = "time+SE2" if args.auto_align_se2 else "time+XY"
        print(f"[INFO] Auto-align {mode} applied: gt_time_offset={offset:.3f}s, "
              f"yaw={yaw:.3f} rad, dx={dx:.3f} m, dy={dy:.3f} m, pairs={n_pairs}")
    elif args.auto_align_se2:
        yaw, dx, dy, residual, n_pairs = estimate_se2_alignment(
            detector_frames,
            person_positions,
            t_start_offset=args.t_start,
            align_window=args.auto_align_window,
            gt_time_offset=effective_gt_time_offset)
        person_positions = transform_positions_xy(person_positions, yaw, dx, dy, 0.0)
        print(f"[INFO] Auto-align SE2 applied: yaw={yaw:.3f} rad, "
              f"dx={dx:.3f} m, dy={dy:.3f} m, residual={residual:.3f} m, pairs={n_pairs}")
    elif args.auto_align_xy:
        dx, dy = estimate_xy_translation(
            detector_frames,
            person_positions,
            t_start_offset=args.t_start,
            align_window=args.auto_align_window,
            gt_time_offset=effective_gt_time_offset)
        person_positions = transform_positions_xy(person_positions, 0.0, dx, dy, 0.0)
        print(f"[INFO] Auto-align XY applied: dx={dx:.3f} m, dy={dy:.3f} m")
    else:
        print(f"[INFO] GT->odom transform: yaw={args.gt_yaw:.3f} rad, "
              f"t=[{args.gt_tx:.3f}, {args.gt_ty:.3f}, {args.gt_tz:.3f}] m")

    # Valuta
    results = evaluate(
        detector_frames   = detector_frames,
        person_positions  = person_positions,
        person_velocities = person_velocities,
        t_start_offset    = args.t_start,
        match_dist_thresh = args.match_dist_thresh,
        dyn_vel_thresh    = args.dyn_vel_thresh,
        static_vel_thresh = args.static_vel_thresh,
        force_no_gt_potentially_dynamic = args.force_no_gt_potentially_dynamic,
        t_max_diff        = args.t_max_diff,
        time_mode         = args.time_mode,
        gt_time_offset    = effective_gt_time_offset,
    )

    if not results:
        print("[ERROR] Nessun frame nella finestra di valutazione.")
        sys.exit(1)

    # Stampa e salva
    print_results(exp_name, results, args.dyn_vel_thresh)
    out_dir = os.path.join(args.output_dir, exp_name)
    save_csv(out_dir, exp_name, results, args.dyn_vel_thresh)

    print(f"\n[DONE] Risultati in: {Path(out_dir).resolve()}")


if __name__ == "__main__":
    main()
