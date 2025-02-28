import matplotlib.pyplot as plt
import math
import time
import heapq
from collections import defaultdict

import numpy as np

from src.position_direction_planning import MapPreprocessing, AStarSearch, Loadmap
from src.utils import *


def queue_element(distance, t, index, parent_index, d_first):
    if not d_first:
        return t, distance, index, parent_index
    if d_first:
        return distance, t, index, parent_index

# def true_distance(edge_index, max_steer_angle, begin, end, begin_angle, end_angle, robot_width):
#     db_reader = Loadmap()
#     map_preprocessor = MapPreprocessing(db_reader, max_steer_angle)
#     # 进行A*搜索，参数为起始点、目标点、起始角度、目标角度和机器人宽度
#     astar_search = AStarSearch(db_reader, map_preprocessor, begin, end, begin_angle, end_angle,
#                                robot_width)
#     end_index = astar_search.edges[edge_index].start_end_index[1]
#     phi_start = astar_search.dictionarys[edge_index].tangent_vector_set[1]
#     print(astar_search.edges[edge_index].edge_index, end_index, phi_start)
#     d = astar_search.search_all_node(end_index, phi_start)
#     return d


class STAStarSearch:
    def __init__(self, preprocessed_map, A_begin, A_end, A_beginAngle, A_endAngle, A_robot_width):
        """
        Python 等价构造函数
        """
        # 初始化外部对象
        self.preprocessed_map = preprocessed_map

        # 初始化地图数据
        # self.time_start_mapload = databaseReader.getTime_start_mapload()
        self.max_steer_angle = preprocessed_map.max_steer_angle
        self.edges = preprocessed_map.edges
        self.nodes = preprocessed_map.nodes
        self.dictionarys = preprocessed_map.dictionarys
        self.successiveStateSet = preprocessed_map.successiveStateSet

        # # 确保初始化搜索开始时间
        # self.time_start_searching = time.time()

        # 初始化搜索参数
        self.edgeNum = len(self.edges)
        self.begin = A_begin
        self.end = A_end
        self.beginAngle = A_beginAngle
        self.endAngle = A_endAngle
        self.robot_width = A_robot_width

        self.end_stage = None
        self.end_time = None

        # 计算起点和终点的方向向量
        self.phi_start = (math.cos(math.radians(self.beginAngle)), math.sin(math.radians(self.beginAngle)))
        self.phi_end = (math.cos(math.radians(self.endAngle)), math.sin(math.radians(self.endAngle)))

        # 初始化路径结果
        self.resultsEdgePath = []
        self.resultsStatePath = []

        self.close_dict = {}
        self.state_info = {}

        for state_index in range(len(self.successiveStateSet)):
            self.state_info[state_index] = {}

    def searching(self, constraint_set=None, d_first=True):
        """
        A* 搜索算法实现
        """
        # 优先队列（基于 heapq），存储 (代价, 状态编号)
        pq = []
        heapq.heapify(pq)
        t = 0

        if constraint_set is None:
            constraint_set = []

        # 遍历所有边，初始化起点的可达状态
        for r in self.edges:
            if r.start_end_index[0] == self.begin:
                t_start = self.dictionarys[r.edge_index].tangent_vector_set[0]
                theta_start = self.preprocessed_map.vector_angle(self.phi_start, t_start)

                # 正向可达状态
                if (theta_start <= self.max_steer_angle
                        and self.dictionarys[r.edge_index].min_radius >= self.preprocessed_map.r_min
                        and self.dictionarys[r.edge_index].road_width > self.robot_width):
                    distance = (self.dictionarys[r.edge_index].forward_cost
                                + self.preprocessed_map.cal_distance(
                                self.nodes[self.edges[r.edge_index].start_end_index[1]], self.nodes[self.end]))
                    # distance = (self.dictionarys[r.edge_index].reverse_cost +
                    #             true_distance(r.edge_index, self.max_steer_angle, self.begin, self.end, self.beginAngle,
                    #                           self.endAngle, self.robot_width))

                    self.successiveStateSet[2 * r.edge_index].distance = distance
                    heapq.heappush(pq, queue_element(distance, t, 2 * r.edge_index, -1, d_first))
                    self.state_info[2 * r.edge_index][t] = -1

                # 反向可达状态
                if (180 - theta_start <= self.max_steer_angle
                        and self.dictionarys[r.edge_index].min_radius >= self.preprocessed_map.r_min
                        and self.dictionarys[r.edge_index].road_width > self.robot_width):
                    distance = (self.dictionarys[r.edge_index].reverse_cost
                                + self.preprocessed_map.cal_distance(
                                self.nodes[self.edges[r.edge_index].start_end_index[1]], self.nodes[self.end]))
                    # distance = (self.dictionarys[r.edge_index].reverse_cost +
                    #             true_distance(r.edge_index, self.max_steer_angle, self.begin, self.end, self.beginAngle,
                    #                           self.endAngle, self.robot_width))

                    self.successiveStateSet[2 * r.edge_index + 1].distance = distance
                    heapq.heappush(pq, queue_element(distance, t, 2 * r.edge_index + 1, -1, d_first))
                    self.state_info[2 * r.edge_index + 1][t] = -1
        count = 0

        # 主循环
        while pq:
            count += 1

            if d_first:
                # 取出优先队列中代价最小的状态
                d, t, u, _ = heapq.heappop(pq)
            else:
                t, d, u, _ = heapq.heappop(pq)
            # edge_index = self.successiveStateSet[u].edge_index

            # print(d, t, u)
            # 在STAstar中需要修改，修改成考虑时间戳的闭集
            if (u, t) in self.close_dict:
                # 如果状态已在闭集中，则跳过
                continue
            else:
                # 否则，将其加入闭集中
                self.close_dict[(u, t)] = {"visited": 1}

            # 获取状态对应的边索引
            u_edge_index = self.successiveStateSet[u].edge_index

            # 判断是否到达终点
            if self.edges[u_edge_index].start_end_index[1] == self.end:
                t_end = self.dictionarys[u_edge_index].tangent_vector_set[1]
                theta_end = self.preprocessed_map.vector_angle(self.phi_end, t_end)

                # 满足终点方向约束时，退出搜索
                if (u % 2 == 0 and theta_end <= self.max_steer_angle) or (
                        u % 2 == 1 and 180 - theta_end <= self.max_steer_angle
                ):
                    self.end_stage = u
                    self.end_time = t
                    break

            is_conflict = False

            visited_time = self.successiveStateSet[u].visited_time
            self.successiveStateSet[u].visited_time = visited_time + 1

            # print(u_edge_index, v_edge_index)
            for constraint in constraint_set:
                constraint_node = constraint.node_index
                constraint_edge = constraint.edge_index
                constraint_t = constraint.t
                # if v_edge_index == constraint_edge:
                if constraint_edge // 2 == u_edge_index // 2:
                    if constraint_t == t + 1:
                        # print("not enqueue edge:", u_edge_index, self.edges[u_edge_index].start_end_index[1], t)
                        is_conflict = True
                        break
                if constraint_node == self.edges[u_edge_index].start_end_index[1]:
                    if constraint_t == t + 1:
                        # print("not enqueue node:", u_edge_index, self.edges[u_edge_index].start_end_index[1], t)
                        is_conflict = True
                        break

            min_h = float('inf')
            # 更新子节点
            # 考虑冲突
            for v in self.successiveStateSet[u].childrenState:
                u_edge_index = self.successiveStateSet[u].edge_index
                v_edge_index = self.successiveStateSet[v].edge_index

                is_conflict = False
                # print(u_edge_index, v_edge_index)
                for constraint in constraint_set:
                    constraint_node = constraint.node_index
                    constraint_edge = constraint.edge_index
                    constraint_t = constraint.t
                    # if v_edge_index == constraint_edge:
                    if constraint_edge//2 == v_edge_index//2:
                        if constraint_t == t+1:
                            # print("not enqueue edge:", v_edge_index, self.edges[v_edge_index].start_end_index[1], t)
                            is_conflict = True
                            break
                    if constraint_node == self.edges[v_edge_index].start_end_index[1]:
                        if constraint_t == t + 1:
                            # print("not enqueue node:", v_edge_index, self.edges[v_edge_index].start_end_index[1], t)
                            is_conflict = True
                            break
                if is_conflict:
                    continue

                u_end = self.nodes[self.edges[u_edge_index].start_end_index[1]]
                v_end = self.nodes[self.edges[v_edge_index].start_end_index[1]]
                goal_node = self.nodes[self.end]
                d_u_to_goal = self.preprocessed_map.cal_distance(u_end, goal_node)
                # d_u_to_goal = true_distance(u_edge_index, self.max_steer_angle, self.begin, self.end, self.beginAngle,
                #                             self.endAngle, self.robot_width)
                d_v_to_goal = self.preprocessed_map.cal_distance(v_end, goal_node)
                # d_v_to_goal = true_distance(v_edge_index, self.max_steer_angle, self.begin, self.end, self.beginAngle,
                #                             self.endAngle, self.robot_width)
                # 计算 g(u)
                g_u = self.successiveStateSet[u].distance - d_u_to_goal

                # 正向子节点
                if (
                        v % 2 == 0
                        and self.successiveStateSet[v].distance
                        > g_u
                        + self.dictionarys[v_edge_index].forward_cost
                        + d_v_to_goal
                        and self.dictionarys[v_edge_index].road_width > self.robot_width
                ):
                    self.successiveStateSet[v].distance = (
                            g_u
                            + self.dictionarys[v_edge_index].forward_cost
                            + d_v_to_goal
                    )
                    if self.successiveStateSet[v].distance < min_h:
                        min_h = self.successiveStateSet[v].distance
                    self.successiveStateSet[v].pre = u
                    heapq.heappush(pq, queue_element(self.successiveStateSet[v].distance, t+1, v, u, d_first))
                    self.state_info[v][t] = u

                # 反向子节点
                if (
                        v % 2 == 1
                        and self.successiveStateSet[v].distance
                        > g_u
                        + self.dictionarys[v_edge_index].reverse_cost
                        + d_v_to_goal
                        and self.dictionarys[v_edge_index].road_width > self.robot_width
                ):
                    self.successiveStateSet[v].distance = (
                            g_u
                            + self.dictionarys[v_edge_index].reverse_cost
                            + d_v_to_goal
                    )
                    if self.successiveStateSet[v].distance < min_h:
                        min_h = self.successiveStateSet[v].distance
                    self.successiveStateSet[v].pre = u
                    heapq.heappush(pq, queue_element(self.successiveStateSet[v].distance, t+1, v, u, d_first))
                    self.state_info[v][t] = u

            if is_conflict is False:
                # 将自己作为下一个时刻可以选择的节点
                ################################################################
                heapq.heappush(pq, queue_element(self.successiveStateSet[u].distance*(1 + 0.5 * visited_time), t + 1, u, u, d_first))
                # heapq.heappush(pq, queue_element(min_h + 0.001, t + 1, u, u, d_first))
                ##################################################################

                self.state_info[u][t] = u

    def process(self, constraint_set=None):
        """
        回溯路径
        """
        if constraint_set is None:
            constraint_set = []
        self.searching(constraint_set)

        if self.end_stage is None:
            print("No path found")
            return [], [], []

        path = []
        current = self.end_stage
        current_time = self.end_time
        # print()
        # print("path state and time:")
        # print(current, current // 2,self.edges[current // 2].start_end_index[1], current_time)

        self.resultsStatePath.insert(0, current)
        self.resultsEdgePath.insert(0, current//2)
        while current != -1 and current_time != 0:
            path.append((current, current_time))
            u = self.state_info[current][current_time-1]  # 获取前驱节点和距离
            # 假设时间是单调倒退的，可以通过某种规则更新 current_time
            current_time -= 1  # 或者根据你的问题逻辑调整时间
            current = u
            if current == self.resultsStatePath[0]:
                self.resultsStatePath.insert(0, -1)
            else:
                self.resultsStatePath.insert(0, current)
            self.resultsEdgePath.insert(0, current//2)

        unique_elements = set(self.resultsStatePath)
        path_cost = len(unique_elements)

        # # 回溯路径
        # self.resultsStatePath.insert(0, self.end_stage)
        # temp_pre = self.successiveStateSet[self.end_stage].pre
        # while temp_pre != -1:
        #     self.resultsStatePath.insert(0, temp_pre)
        #     temp_pre = self.successiveStateSet[temp_pre].pre
        #
        # for state in self.resultsStatePath:
        #     self.resultsEdgePath.append(self.successiveStateSet[state].edge_index)

        return self.resultsEdgePath, self.resultsStatePath, path_cost