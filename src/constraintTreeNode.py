from typing import Dict
import numpy as np
from src.utils import Constraint


class CTNode:
    def __init__(self, constraints, solutions):
        self.constraints = constraints
        self.solutions = solutions
        self.cost = self.sum_of_steps(solutions)

    @staticmethod
    def sum_of_steps(solutions):
        """
        计算所有solution中step的总和，不考虑路径长度的cost，只考虑decision step的数量
        :param solutions:
        :return:
        """
        cost = 0
        for i in range(len(solutions)):
            if not solutions[i]['path']:
                # 初始化的空list代表没有solution，赋予最大的cost，只在第一次调用
                return np.inf
            else:
                cost = cost + solutions[i]['path_cost']

        return cost

    def __lt__(self, other):
        return self.cost < other.cost

    def __str__(self):
        return str(self.constraints.agent_constraints)
