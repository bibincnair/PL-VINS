#!/usr/bin/env python3
"""Replay a LaMAria ROS 2 MCAP sequence and evaluate it with EVO."""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


def quaternion_matrix(q):
    # Internal poses use [w, x, y, z].
    w, x, y, z = q
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm == 0:
        raise ValueError('zero-length calibration quaternion')
    w, x, y, z = (v / norm for v in (w, x, y, z))
    return [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]


def write_config(calibration_path, output_path, loop_closure=False):
    calibration = json.loads(calibration_path.read_text())
    camera = calibration['cam0']
    imu = calibration['imu0']
    fx, fy, cx, cy = camera['params'][:4]
    width = int(camera['resolution']['width'])
    height = int(camera['resolution']['height'])
    extrinsic = camera['T_b_s']
    q_xyzw = extrinsic['qvec']
    rotation = quaternion_matrix([q_xyzw[3], *q_xyzw[:3]])
    translation = extrinsic['tvec']
    rate = float(imu['imu_rate'])
    # PL-VINS expects per-sample standard deviations; LaMAria supplies Kalibr
    # continuous-time noise densities, so convert with sqrt(samples/second).
    scale = math.sqrt(rate)
    values = {
        'acc_n': float(imu['acc_noise_density']) * scale,
        'gyr_n': float(imu['gyro_noise_density']) * scale,
        'acc_w': float(imu['acc_bias_random_walk_sigma']) * scale,
        'gyr_w': float(imu['gyro_bias_random_walk_sigma']) * scale,
        'g_norm': float(imu['gravity_magnitude']),
    }
    mat = lambda rows: ',\n      '.join(', '.join(f'{v:.12g}' for v in row) for row in rows)
    text = f'''%YAML:1.0
imu_topic: "/imu0"
image_topic: "/cam0/image_raw"
model_type: PINHOLE
camera_name: cam0
image_width: {width}
image_height: {height}
distortion_parameters:
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
projection_parameters:
  fx: {fx:.12g}
  fy: {fy:.12g}
  cx: {cx:.12g}
  cy: {cy:.12g}
estimate_extrinsic: 0
extrinsicRotation: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [{mat(rotation)}]
extrinsicTranslation: !!opencv-matrix
  rows: 3
  cols: 1
  dt: d
  data: [{', '.join(f'{v:.12g}' for v in translation)}]
max_cnt: 150
min_dist: 30
freq: 0
F_threshold: 1.0
show_track: 1
equalize: 1
fisheye: 0
max_solver_time: 0.06
max_num_iterations: 10
keyframe_parallax: 10.0
acc_n: {values['acc_n']:.12g}
gyr_n: {values['gyr_n']:.12g}
acc_w: {values['acc_w']:.12g}
gyr_w: {values['gyr_w']:.12g}
g_norm: {values['g_norm']:.12g}
loop_closure: {1 if loop_closure else 0}
load_previous_pose_graph: 0
fast_relocalization: 0
estimate_td: 0
td: 0.0
rolling_shutter: 0
rolling_shutter_tr: 0
save_image: 0
visualize_imu_forward: 0
visualize_camera_size: 0.4
'''
    output_path.write_text(text)
    return extrinsic


def read_poses(path):
    poses = []
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        line = line.replace(',', ' ').strip()
        if not line or line.startswith('#'):
            continue
        values = [float(v) for v in line.split()]
        if len(values) != 8:
            raise ValueError(f'{path}:{line_number}: expected 8 pose values')
        poses.append(values)
    if len(poses) < 3:
        raise ValueError(f'{path}: fewer than three poses')
    if any(b[0] <= a[0] for a, b in zip(poses, poses[1:])):
        raise ValueError(f'{path}: timestamps must increase strictly')
    return poses


def write_reference(source, output):
    poses = read_poses(source)
    with output.open('w') as stream:
        for row in poses:
            # LaMAria uses nanoseconds; EVO's TUM format uses seconds.
            stream.write(f'{row[0] / 1e9:.9f} ' + ' '.join(f'{v:.12g}' for v in row[1:]) + '\n')


def transform_estimate(source, output, extrinsic):
    """Convert PL-VINS world_from_imu poses to LaMAria world_from_cam0 poses."""
    poses = read_poses(source)
    q_ic = extrinsic['qvec']  # LaMAria stores calibration quaternions as x, y, z, w.
    t_ic = extrinsic['tvec']
    with output.open('w') as stream:
        for row in poses:
            t = row[1:4]
            q_wi = [row[7], row[4], row[5], row[6]]
            r_wi = quaternion_matrix(q_wi)
            t_wc = [t[i] + sum(r_wi[i][j] * t_ic[j] for j in range(3)) for i in range(3)]
            # Hamilton product q_wc = q_wi * q_ic.
            w, x, y, z = q_wi
            b, c, d, a = q_ic
            q_wc = [w*a-x*b-y*c-z*d, w*b+x*a+y*d-z*c,
                    w*c-x*d+y*a+z*b, w*d+x*c-y*b+z*a]
            norm = math.sqrt(sum(v*v for v in q_wc))
            q_wc = [v / norm for v in q_wc]
            tum = [row[0], *t_wc, q_wc[1], q_wc[2], q_wc[3], q_wc[0]]
            stream.write(' '.join(f'{v:.12g}' for v in tum) + '\n')


def write_submission(source, output):
    """Write LaMAria's nanosecond, world_from_imu pose format."""
    poses = read_poses(source)
    with output.open('w') as stream:
        for row in poses:
            timestamp_ns = round(row[0] * 1e9)
            stream.write(f'{timestamp_ns} ' + ' '.join(f'{v:.12g}' for v in row[1:]) + '\n')


def terminate(process):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=15)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5)


def wait_for_nodes(process, log_path, timeout):
    required = {'/feature_tracker', '/linefeature_tracker', '/plvins_estimator'}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f'PL-VINS launch exited early; see {log_path}')
        result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
        if required.issubset(set(result.stdout.splitlines())):
            return
        time.sleep(1)
    raise TimeoutError(f'PL-VINS nodes did not become ready within {timeout}s; see {log_path}')


def run_evo(reference, estimate, out_dir):
    plot_config = out_dir / 'evo_headless_config.json'
    plot_config.write_text(json.dumps({'plot_backend': 'Agg'}, indent=2) + '\n')
    commands = [
        ('ape', ['evo_ape', 'tum', str(reference), str(estimate), '--align',
                 '--save_results', str(out_dir / 'ape.zip'),
                 '--save_plot', str(out_dir / 'ape.png'), '--config', str(plot_config)]),
        ('rpe_translation', ['evo_rpe', 'tum', str(reference), str(estimate), '--align',
                 '--pose_relation', 'trans_part', '--delta', '1', '--delta_unit', 'm',
                 '--save_results', str(out_dir / 'rpe_translation.zip'),
                 '--save_plot', str(out_dir / 'rpe_translation.png'),
                 '--config', str(plot_config)]),
        ('rpe_rotation', ['evo_rpe', 'tum', str(reference), str(estimate), '--align',
                 '--pose_relation', 'angle_deg', '--delta', '1', '--delta_unit', 'm',
                 '--save_results', str(out_dir / 'rpe_rotation.zip'),
                 '--save_plot', str(out_dir / 'rpe_rotation.png'),
                 '--config', str(plot_config)]),
    ]
    # EVO asks Matplotlib to render its plots, but force a noninteractive
    # backend so this also works on headless machines and never opens windows.
    env = os.environ.copy()
    env['MPLBACKEND'] = 'Agg'
    for name, command in commands:
        result = subprocess.run(command, text=True, capture_output=True, env=env)
        (out_dir / f'{name}.txt').write_text(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(f"{command[0]} failed; see {out_dir / f'{name}.txt'}")
        missing = [f'{name}_{figure}.png' for figure in ('raw', 'map')
                   if not (out_dir / f'{name}_{figure}.png').is_file()]
        if missing:
            raise RuntimeError(f"{command[0]} did not save expected plots: {', '.join(missing)}")


def create_run_dir(output_root, sequence, playback_rate, loop_closure, run_label=None):
    """Create a collision-safe, descriptive output directory for one replay."""
    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    rate = format(playback_rate, 'g').replace('.', 'p')
    parts = [timestamp]
    if run_label:
        label = re.sub(r'[^A-Za-z0-9._-]+', '-', run_label).strip('-._')
        if not label:
            raise ValueError('--run-label must include a letter or number')
        parts.append(label)
    parts.extend((f'rate-{rate}', f'lc-{int(loop_closure)}'))
    base_name = '_'.join(parts)
    sequence_root = output_root / sequence
    sequence_root.mkdir(parents=True, exist_ok=True)
    for suffix in range(1000):
        run_name = base_name if suffix == 0 else f'{base_name}_{suffix:03d}'
        out_dir = sequence_root / run_name
        try:
            out_dir.mkdir()
        except FileExistsError:
            continue
        return out_dir
    raise RuntimeError(f'could not allocate a unique run directory under {sequence_root}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset-root', type=Path, default=Path('/datasets/lamaraia'))
    parser.add_argument('--sequence', required=True, help='e.g. R_01_easy')
    parser.add_argument('--bag', type=Path, help='bag directory (defaults to SEQUENCE/SEQUENCE_0.mcap parent)')
    parser.add_argument('--calibration', type=Path, help='pinhole calibration JSON; defaults to SEQUENCE/SEQUENCE.json')
    parser.add_argument('--groundtruth', type=Path, help='dense pGT text; defaults to SEQUENCE/SEQUENCE.txt')
    parser.add_argument('--output-root', type=Path, default=Path('results/lamaria'))
    parser.add_argument('--run-label', help='optional short label included in this run folder name')
    parser.add_argument('--playback-rate', type=float, default=0.5)
    parser.add_argument('--startup-timeout', type=int, default=90)
    parser.add_argument('--loop-closure', action='store_true')
    parser.add_argument('--no-evaluate', action='store_true', help='run SLAM but skip EVO metrics')
    args = parser.parse_args()

    if args.playback_rate <= 0:
        parser.error('--playback-rate must be positive')
    sequence_dir = args.dataset_root / args.sequence
    calibration = args.calibration or sequence_dir / f'{args.sequence}.json'
    groundtruth = args.groundtruth or sequence_dir / f'{args.sequence}.txt'
    bag_dir = args.bag or sequence_dir
    bag_files = list(bag_dir.glob('*.mcap')) if bag_dir.is_dir() else []
    if not bag_files:
        parser.error(f'no .mcap bag found under {bag_dir}')
    if not calibration.is_file() or not groundtruth.is_file():
        parser.error(f'missing calibration or ground truth for {args.sequence}')
    if not shutil.which('ros2'):
        parser.error('ros2 not found; source tools/setup.bash first')
    if not args.no_evaluate and any(not shutil.which(name) for name in ('evo_ape', 'evo_rpe')):
        parser.error('EVO CLI not found; install requirements-evo.lock first or pass --no-evaluate')

    # OpenCV is installed privately by the PL-VINS dependency bootstrap. The
    # executables have RUNPATHs for direct dependencies, but transitive OpenCV
    # dependencies also need this directory in LD_LIBRARY_PATH.
    env = os.environ.copy()
    vendor_lib = Path(__file__).resolve().parents[1] / '.deps' / 'install' / 'lib'
    if vendor_lib.is_dir():
        prior = env.get('LD_LIBRARY_PATH', '')
        env['LD_LIBRARY_PATH'] = str(vendor_lib) + (os.pathsep + prior if prior else '')

    try:
        out_dir = create_run_dir(args.output_root, args.sequence, args.playback_rate,
                                 args.loop_closure, args.run_label)
        metadata = {
            'run_id': out_dir.name,
            'started_utc': datetime.now(timezone.utc).isoformat(),
            'sequence': args.sequence,
            'dataset_root': str(args.dataset_root.resolve()),
            'bag': str(bag_dir.resolve()),
            'calibration': str(calibration.resolve()),
            'groundtruth': str(groundtruth.resolve()),
            'playback_rate': args.playback_rate,
            'loop_closure': args.loop_closure,
            'evaluation_enabled': not args.no_evaluate,
        }
        (out_dir / 'run.json').write_text(json.dumps(metadata, indent=2) + '\n')
        extrinsic = write_config(calibration, out_dir / 'config.yaml', args.loop_closure)
        reference = out_dir / 'reference_cam0_tum.txt'
        write_reference(groundtruth, reference)
        launch = ['ros2', 'launch', 'plvins_estimator', 'plvins.launch.py',
                  f'config_file:={out_dir / "config.yaml"}', f'output_directory:={out_dir}',
                  'use_sim_time:=true', f'loop_closure:={str(args.loop_closure).lower()}']
        with (out_dir / 'launch.log').open('w') as launch_log:
            process = subprocess.Popen(launch, stdout=launch_log, stderr=subprocess.STDOUT,
                                       start_new_session=True, env=env)
            try:
                wait_for_nodes(process, out_dir / 'launch.log', args.startup_timeout)
                with (out_dir / 'bag.log').open('w') as bag_log:
                    playback = subprocess.run(['ros2', 'bag', 'play', str(bag_dir), '--clock',
                                               '--rate', str(args.playback_rate)],
                                              stdout=bag_log, stderr=subprocess.STDOUT, env=env)
                if playback.returncode:
                    raise RuntimeError(f'ros2 bag play failed; see {out_dir / "bag.log"}')
                time.sleep(2)
            finally:
                terminate(process)
        estimate = out_dir / 'trajectory_tum.txt'
        if not estimate.is_file() or estimate.stat().st_size == 0:
            raise RuntimeError(f'PL-VINS produced no trajectory; see {out_dir / "launch.log"}')
        camera_estimate = out_dir / 'trajectory_cam0_tum.txt'
        transform_estimate(estimate, camera_estimate, extrinsic)
        write_submission(estimate, out_dir / f'{args.sequence}.txt')
        if not args.no_evaluate:
            run_evo(reference, camera_estimate, out_dir)
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError, json.JSONDecodeError) as error:
        print(f'run_lamaria: {error}', file=sys.stderr)
        return 1
    print(f'Run artifacts: {out_dir}')
    if not args.no_evaluate:
        print(f'EVO APE: {out_dir / "ape.txt"}; plots {out_dir / "ape_raw.png"}, '
              f'{out_dir / "ape_map.png"}')
        print(f'EVO RPE: {out_dir / "rpe_translation.txt"} and {out_dir / "rpe_rotation.txt"}; '
              f'plots {out_dir / "rpe_translation_raw.png"}, {out_dir / "rpe_translation_map.png"}, '
              f'{out_dir / "rpe_rotation_raw.png"}, {out_dir / "rpe_rotation_map.png"}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
