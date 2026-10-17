#!/usr/bin/env python3
"""Compute rigid-aligned position ATE for TUM estimates against EuRoC ground-truth CSV."""
import argparse
import json
from pathlib import Path
import numpy as np


def evaluate(estimate, reference):
    if estimate.ndim!=2 or estimate.shape[1]!=8 or reference.ndim!=2 or reference.shape[1]<4:
        raise ValueError('expected TUM estimate and EuRoC ground-truth CSV')
    if not np.isfinite(estimate).all() or not np.isfinite(reference[:, :4]).all():
        raise ValueError('nonfinite trajectory data')
    reference=reference.copy(); reference[:,0]*=1e-9
    if not np.all(np.diff(estimate[:,0])>0) or not np.all(np.diff(reference[:,0])>0):
        raise ValueError('timestamps must increase strictly')
    valid=(estimate[:,0]>=reference[0,0])&(estimate[:,0]<=reference[-1,0])
    estimate=estimate[valid]
    if len(estimate)<3: raise ValueError('fewer than three overlapping poses')
    ground_truth=np.column_stack([np.interp(estimate[:,0],reference[:,0],reference[:,i]) for i in [1,2,3]])
    x=estimate[:,1:4];x=x-x.mean(axis=0);y=ground_truth-ground_truth.mean(axis=0)
    u,_,vt=np.linalg.svd(x.T@y);sign=np.eye(3);sign[2,2]=np.linalg.det(u@vt)
    errors=np.linalg.norm(x@(u@sign@vt)-y,axis=1)
    return {'poses':len(estimate),'duration_seconds':float(estimate[-1,0]-estimate[0,0]),
            'alignment':'rigid SE(3), no scale fit','ate_rmse_m':float(np.sqrt(np.mean(errors**2))),
            'ate_median_m':float(np.median(errors)),'ate_max_m':float(errors.max())}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('estimate',type=Path)
    parser.add_argument('reference',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    try:
        result=evaluate(np.loadtxt(args.estimate,ndmin=2),np.loadtxt(args.reference,delimiter=',',comments='#',ndmin=2))
    except (ValueError,OSError) as error:parser.error(str(error))
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    print(text,end='')


if __name__=='__main__':main()
