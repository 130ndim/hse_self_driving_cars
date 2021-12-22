# Log at https://github.com/OSLL/aido-auto-feedback/tree/ec893044b30926269dd369d71b2abb857b3eb04c9ea09922b7db2466/dont_crush_duckie

from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

import logging

logger = logging.getLogger(__name__)


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)
        self.env = generated_task['env']

    def solve(self):
        # getting the initial picture
        img, _, _, _ = self.env.step([0, 0])

        condition = True
        while condition:
            img, reward, done, info = self.env.step([1, 0])

            img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
            lower = np.array([20, 100, 100], dtype="uint8")
            upper = np.array([30, 255, 255], dtype="uint8")
            mask = (cv2.inRange(img, lower, upper) > 0)

            pct_yellow = 100 * (mask.sum() / np.prod(img.shape))
            print("Yellow pixels: %.2f%%" % pct_yellow)

            if pct_yellow > 2.:
                self.move_left()
                self.move_right()
                self.move_forward(100)
                condition = False

            self.env.render()

    def move_left(self):
        for _ in range(20):
            __ = self.env.step([0, 1])
            self.env.render()

        self.move_forward()

        for _ in range(20):
            __ = self.env.step([0, -1])
            self.env.render()

    def move_forward(self, steps=10):
        for _ in range(steps):
            __ = self.env.step([1, 0])
            self.env.render()

    def move_right(self):
        for _ in range(20):
            __ = self.env.step([0, -1])
            self.env.render()

        self.move_forward()

        for _ in range(20):
            __ = self.env.step([0, 1])
            self.env.render()
