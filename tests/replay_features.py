#!/usr/bin/env python3
"""Replay an external ROS 2 feature/IMU bag through the installed estimator."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag',type=Path)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rate',type=float,default=2.)
    parser.add_argument('--timeout',type=float,default=600.)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env['ROS_DOMAIN_ID']='218';env['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST';env.pop('ROS_LOCALHOST_ONLY',None)
    node=None;player=None
    with (args.output/'estimator.log').open('w') as log,(args.output/'player.log').open('w') as player_log:
        try:
            node=subprocess.Popen(['ros2','run','plvins_estimator','plvins_estimator','--ros-args',
                '-p','config_file:='+str(args.config.resolve()),'-p','output_directory:='+str(args.output.resolve())],
                env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            time.sleep(2)
            if node.poll() is not None: raise RuntimeError('estimator failed to start')
            player=subprocess.Popen(['ros2','bag','play',str(args.bag),'--rate',str(args.rate),'--delay','1'],
                env=env,stdout=player_log,stderr=subprocess.STDOUT,start_new_session=True)
            end=time.monotonic()+args.timeout
            while player.poll() is None:
                if node.poll() is not None: raise RuntimeError('estimator crashed during replay')
                if time.monotonic()>end: raise TimeoutError('replay exceeded timeout')
                time.sleep(.25)
            if player.returncode!=0: raise RuntimeError('bag playback failed')
            time.sleep(3)
        finally:
            for process in [player,node]:
                if process is not None and process.poll() is None:
                    os.killpg(process.pid,signal.SIGINT)
                    try: process.wait(timeout=15)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid,signal.SIGKILL);process.wait()
                        raise RuntimeError('process did not shut down')
    if node.returncode not in (0,130):raise RuntimeError(f'estimator exited with {node.returncode}')
    import numpy as np
    trajectory=np.loadtxt(args.output/'trajectory_tum.txt',ndmin=2)
    if trajectory.shape[0]<10 or trajectory.shape[1]!=8 or not np.isfinite(trajectory).all():
        raise RuntimeError('no valid estimated trajectory')
    if not np.all(np.diff(trajectory[:,0])>0):raise RuntimeError('trajectory timestamps did not increase')
    print(f'Replay passed: {trajectory.shape[0]} finite poses; output {args.output}')


if __name__=='__main__':main()
