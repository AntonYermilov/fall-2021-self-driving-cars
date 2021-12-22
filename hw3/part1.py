from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2


def count_yellow(img):
    yellow = np.array([220, 220, 30])
    dist = np.linalg.norm((img - yellow) / 255.0, axis=2) / np.sqrt(3)
    return np.sum(dist < 0.3)


class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img0, _, _, _ = env.step([0, 0])
        n, m, _ = img0.shape
        cnt0 = count_yellow(img0)

        condition = True
        while condition:
            img, _, _, _ = env.step([1, 0])
            cnt = count_yellow(img)
            condition = ((cnt - cnt0) / (n * m)) < 0.0691
            env.render()
