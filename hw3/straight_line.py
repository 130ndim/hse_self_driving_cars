# Log at https://github.com/OSLL/aido-auto-feedback/tree/a6a6958148a3bbca2cf6a88349169cea76948f55b8b17e079c254daa/dont_crush_duckie

from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

import logging

logger = logging.getLogger(__name__)


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0, 0])

        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])

            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            lower = np.array([20, 100, 100], dtype="uint8")
            upper = np.array([30, 255, 255], dtype="uint8")
            mask = (cv2.inRange(img, lower, upper) > 0)

            pct_yellow = 100 * (mask.sum() / np.prod(img.shape))
            print("Yellow pixels: %.2f%%" % pct_yellow)

            if pct_yellow > 2.:
                condition = False

            env.render()
