#!/usr/bin/env python3
"""Check the installed simulation launch against an external PL-VINS simulation directory."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('data',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=33.)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env['ROS_DOMAIN_ID']='219';env['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST';env.pop('ROS_LOCALHOST_ONLY',None)
    with (args.output/'launch.log').open('w') as log:
        process=subprocess.Popen(['ros2','launch','plvins_estimator','simulation.launch.py',
            'data_directory:='+str(args.data.resolve()),'output_directory:='+str(args.output.resolve())],
            env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            end=time.monotonic()+args.seconds
            while time.monotonic()<end and process.poll() is None:time.sleep(.25)
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait();raise RuntimeError('simulation shutdown timeout')
    log=(args.output/'launch.log').read_text()
    if 'process has died' in log or '[FATAL]' in log or process.returncode not in (0,130):
        raise RuntimeError('Simulation failed; see '+str(args.output/'launch.log'))
    import numpy as np
    trajectory=np.loadtxt(args.output/'trajectory_tum.txt',ndmin=2)
    if trajectory.shape[0]<10 or trajectory.shape[1]!=8 or not np.isfinite(trajectory).all():
        raise RuntimeError('Simulation produced no valid estimated trajectory')
    print(f'Simulation passed: {len(trajectory)} finite poses; clean shutdown')


if __name__=='__main__':main()
