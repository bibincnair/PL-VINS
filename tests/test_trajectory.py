import importlib.util
from pathlib import Path
import unittest
import numpy as np

spec=importlib.util.spec_from_file_location('evaluate_trajectory',Path(__file__).resolve().parents[1]/'tools/evaluate_trajectory.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class TrajectoryEvaluation(unittest.TestCase):
    def test_rigid_alignment_does_not_fit_scale(self):
        times=np.arange(6,dtype=float)+1
        positions=np.array([[0,0,0],[1,0,0],[1,1,0],[1,1,1],[2,1,1],[3,2,1]],dtype=float)
        rotation=np.array([[0,-1,0],[1,0,0],[0,0,1]],dtype=float)
        reference=np.column_stack([times*1e9,positions])
        estimate=np.column_stack([times,positions@rotation+np.array([5,6,7]),np.tile([0,0,0,1],(6,1))])
        self.assertLess(module.evaluate(estimate,reference)['ate_rmse_m'],1e-12)
        estimate[:,1:4]*=2
        self.assertGreater(module.evaluate(estimate,reference)['ate_rmse_m'],.1)

if __name__=='__main__':unittest.main()
