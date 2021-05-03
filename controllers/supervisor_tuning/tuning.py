from bayes_opt import BayesianOptimization, UtilityFunction
from bayes_opt.logger import JSONLogger
from bayes_opt.event import Events
import os
import numpy as np
from pathlib import Path
class Bayes_opt(object):
    def __init__(self, param):

        ration = 0.5
        ration_h = 1 + ration
        ration_l = 1 - ration
        self.utility = UtilityFunction(kind="ucb", kappa=2.5, xi=0.0)
        self.optimizer = BayesianOptimization(
                            f=None,
                            pbounds={"k_vertical_thrust": (0 if param['k_vertical_thrust']*ration_l <=0 else param['k_vertical_thrust']*ration_l, param['k_vertical_thrust']*ration_h),
                                     "k_vertical_offset": (0 if param['k_vertical_offset']*ration_l <= 0 else param['k_vertical_offset']*ration_l, param['k_vertical_offset']*ration_h),
                                     "k_vertical_p": (0 if param['k_vertical_p']*ration_l <= 0 else param['k_vertical_p']*ration_l, param['k_vertical_p']*ration_h),
                                     "k_roll_p": (0 if param['k_roll_p']*ration_l <= 0 else param['k_roll_p']*ration_l, param['k_roll_p']*ration_h),
                                     "k_pitch_p": (0 if param['k_pitch_p']*ration_l <= 0 else param['k_pitch_p']*ration_l, param['k_pitch_p']*ration_h)},
                            verbose=2,
                            random_state=5,
                        )
        self.log_path = "../../log"
        self.pose_path = "../../pose"
        Path(self.log_path).mkdir(parents=True, exist_ok=True)
        Path(self.pose_path).mkdir(parents=True, exist_ok=True)
        logger = JSONLogger(path=os.path.join(self.log_path, "log.json"))
        self.optimizer.subscribe(Events.OPTIMIZATION_STEP, logger)

    def suggest(self):
        next_params = self.optimizer.suggest(self.utility)
        return next_params

    def optimize(self, params, target):
        self.optimizer.register(params=params, target=target)

    def save_pose_to_log(self, pose, iteration):
        np.savetxt(os.path.join(self.pose_path, "itr_" + str(iteration).zfill(4) + "_pose.txt"), pose)

    def best_suggest(self):
        return self.optimizer.max['params']
