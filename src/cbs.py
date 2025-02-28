import copy
import heapq

from src.position_direction_planning import Loadmap, MapPreprocessing, AStarSearch
from src.Space_Time_AStarSearch import STAStarSearch
from src.utils import *
from src.agent import *
from src.constraintTreeNode import *

import time


class CBS:
    def __init__(self):
        self.robot_width = .1  # 机器人宽度（满载和空载机器人宽度不同）

        self.CTNodes = []
        heapq.heapify(self.CTNodes)

        self.FOCAL = []
        heapq.heapify(self.FOCAL)

        self.result_path = []
        self.result_direction = []
        self.constraint_list = []
        self.solution_list = []

        self.db_reader = Loadmap()
        # 预处理地图，参数alpha_max
        # 为不同动力学参数的agent建立各自的预处理地图
        self.mp = MapPreprocessing(self.db_reader, 20)

        self.max_steer_angle_1 = 20  # 最大转向角, 对应文章中的alpha_max
        self.max_steer_angle_2 = 360
        self.mp1 = MapPreprocessing(self.db_reader, self.max_steer_angle_1)
        self.mp2 = MapPreprocessing(self.db_reader, self.max_steer_angle_2)

        self.mps = {self.max_steer_angle_1: self.mp1,
                    self.max_steer_angle_2: self.mp2}

        self.agents = []
        # self.init_nodes()

    def init_nodes(self, tasks):
        num_tasks = len(tasks)
        self.num_agents = num_tasks

        for i in range(num_tasks):
            task = tasks[i]
            begin = task[0]  # 起始点
            end = task[1]  # 目标点
            begin_angle = task[2]  # 起始角度
            end_angle = task[3]  # 目标角度
            max_steer_angle = task[4]

            self.agents.append(Agent((begin, begin_angle), (end, end_angle), max_steer_angle))
            # print(self.agents[i])
            self.constraint_list.append([])
            self.result_path.append([])
            self.result_direction.append([])
            self.solution_list.append({'agent': i, 'path': []})

        ct_node = CTNode(self.constraint_list, self.solution_list)
        solution_list = self.find_solution(ct_node.constraints)
        conflict = self.find_earliest_conflict(solution_list)
        ct_node.solutions = solution_list

        self.constraint_list = copy.deepcopy(ct_node.constraints)
        self.solution_list = copy.deepcopy(solution_list)

        heapq.heappush(self.CTNodes, (ct_node.cost, ct_node))

        return conflict

    def create_ct_node(self, conflict):
        time = conflict['time']
        agent_1 = conflict['agent1']
        agent_2 = conflict['agent2']
        conflict_type = conflict['type']
        value = conflict['value']

        # 给第一个冲突的agent创造CTNode
        tmp_constraint_list_1 = copy.deepcopy(self.constraint_list)
        if conflict_type == 'edge':
            node = self.mp.edges[value[0]].start_end_index[1]
            constraint_ = Constraint(node, value[0], time)
            tmp_constraint_list_1[agent_1].append(constraint_)
            CTNode(tmp_constraint_list_1, self.solution_list)
        elif conflict_type == 'node':
            node = value[0]
            constraint_ = Constraint(node, -1, time)
            tmp_constraint_list_1[agent_1].append(constraint_)
        else:
            print("wrong conflict type")

        solution_list_1 = copy.deepcopy(self.solution_list)
        solution_list_tmp_1 = self.find_solution(tmp_constraint_list_1, agent_1)
        for solution in solution_list_1:
            # 检查当前 solution 的 agent 索引是否与新 solution 的 agent 索引相同
            if solution['agent'] == solution_list_tmp_1[0]['agent']:
                # 若相同，更新当前 solution 的 path
                solution['path'] = solution_list_tmp_1[0]['path']
                solution['path_cost'] = solution_list_tmp_1[0]['path_cost']

        ct_node_1 = CTNode(tmp_constraint_list_1, solution_list_1)

        # 给第二个冲突的agent创造CTNode
        tmp_constraint_list_2 = copy.deepcopy(self.constraint_list)
        if conflict_type == 'edge':
            node = self.mp.edges[value[1]].start_end_index[1]
            constraint_ = Constraint(node, value[1], time)
            tmp_constraint_list_2[agent_2].append(constraint_)
            CTNode(tmp_constraint_list_2, self.solution_list)
        elif conflict_type == 'node':
            node = value[1]
            constraint_ = Constraint(node, -1, time)
            tmp_constraint_list_2[agent_2].append(constraint_)
        else:
            print("wrong conflict type")

        solution_list_2 = copy.deepcopy(self.solution_list)
        solution_list_tmp_2 = self.find_solution(tmp_constraint_list_2, agent_2)
        for solution in solution_list_2:
            # 检查当前 solution 的 agent 索引是否与新 solution 的 agent 索引相同
            if solution['agent'] == solution_list_tmp_2[0]['agent']:
                # 若相同，更新当前 solution 的 path
                solution['path'] = solution_list_tmp_2[0]['path']
                solution['path_cost'] = solution_list_tmp_2[0]['path_cost']
        ct_node_2 = CTNode(tmp_constraint_list_2, solution_list_2)

        ## DFS
        # if ct_node_2.cost > ct_node_1.cost:
        #     heapq.heappush(self.CTNodes, (ct_node_2.cost, ct_node_2))
        # else:
        #     heapq.heappush(self.CTNodes, (ct_node_1.cost, ct_node_1))
        # c, ct_node_priority = heapq.heappop(self.CTNodes)

        ## CBS
        # heapq.heappush(self.CTNodes, (ct_node_2.cost, ct_node_2))
        # heapq.heappush(self.CTNodes, (ct_node_1.cost, ct_node_1))
        # c, ct_node_priority = heapq.heappop(self.CTNodes)

        ## FOCAL
        # 创建FOCAL列表，然后选择这之中conflict最少的，conflict在放入FOCAL堆中的时候进行计算
        heapq.heappush(self.CTNodes, (ct_node_2.cost, ct_node_2))
        heapq.heappush(self.CTNodes, (ct_node_1.cost, ct_node_1))
        _, ct_node_priority = self.select_from_focal_list(ct_node_1, ct_node_2)


        conflict = self.find_earliest_conflict(ct_node_priority.solutions)
        self.constraint_list = copy.deepcopy(ct_node_priority.constraints)
        self.solution_list = copy.deepcopy(ct_node_priority.solutions)

        # print("conflict", conflict)
        # for i in range(self.num_agents):
        #     print("constraint for agent ", i)
        #     print(self.constraint_list[i])

        return conflict

    def select_from_focal_list(self, ct_node1, ct_node2):
        # focal_list = []
        flag = True
        w = 2
        low_bound, ct_node_optimal = heapq.heappop(self.CTNodes)
        heapq.heappush(self.CTNodes, (low_bound, ct_node_optimal))

        # tmp_focal = []
        # while self.FOCAL:
        #     conflict_num, ct_node = heapq.heappop(self.FOCAL)
        #     cost = ct_node.cost
        #     if cost <= low_bound * w:
        #         heapq.heappush(tmp_focal, (conflict_num, ct_node))
        #
        # self.FOCAL = tmp_focal

        # focal_list.append(ct_node_optimal)

        # while flag and self.CTNodes:
        #     new_cost, ct_node_suboptimal = heapq.heappop(self.CTNodes)
        #     if new_cost <= low_bound * w:
        #         focal_list.append(ct_node_suboptimal)
        #         flag = True
        #     else:
        #         flag = False
        # conflict_num_list = []
        # for focal_node in focal_list:
        #     # 找到每个node中solution的总conflict数量
        #     solution_list_tmp = focal_node.solutions
        #     conflict_num = self.count_conflicts(solution_list_tmp)
        #     conflict_num_list.append(conflict_num)

        if ct_node1.cost <= low_bound * w:
            heapq.heappush(self.FOCAL, (self.count_conflicts(ct_node1.solutions), ct_node1))

        if ct_node2.cost <= low_bound * w:
            heapq.heappush(self.FOCAL, (self.count_conflicts(ct_node2.solutions), ct_node2))


        # i = conflict_num_list.index(min(conflict_num_list))
        # ct_node_priority = focal_list[i]
        c, ct_node_priority = heapq.heappop(self.FOCAL)

        return c, ct_node_priority

    def find_solution(self, constraint_list, constraint_agent=None):
        """
        找到满足当前constraints的一组solution, solution中使用state表示
        :param constraint_list:
        :param constraint_agent: constraint agent 的 index
        :return: solution_list
        """
        solution_list = []

        if constraint_agent is not None:
            # self.mp.preprocessing()
            map_preprocessor = self.mps.get(self.agents[constraint_agent].max_steering_angle)
            map_preprocessor.preprocessing()

            # 进行A*搜索，参数为起始点、目标点、起始角度、目标角度和机器人宽度
            astar_search = STAStarSearch(map_preprocessor, self.agents[constraint_agent].start_index,
                                         self.agents[constraint_agent].goal_index, self.agents[constraint_agent].start_angle,
                                         self.agents[constraint_agent].goal_angle, self.robot_width)
            # 执行A*搜索所需要的步骤，results_path为最终规划的结果
            constraint_set = constraint_list[constraint_agent]

            t1 = time.time()
            results_path, results_state_path, path_cost = astar_search.process(constraint_set)
            t2 = time.time()
            # print("astar search time cost", t2 - t1)
            solution = {'agent': constraint_agent, 'path': [], 'path_cost': path_cost}
            for s_i in results_state_path:
                solution['path'].append(s_i)

            solution_list.append(solution)
        else:
            for i in range(self.num_agents):
                # 重新初始化state
                # self.mp.preprocessing()
                # map_preprocessor = self.mp
                map_preprocessor = self.mps.get(self.agents[i].max_steering_angle)
                map_preprocessor.preprocessing()

                # 进行A*搜索，参数为起始点、目标点、起始角度、目标角度和机器人宽度
                astar_search = STAStarSearch(map_preprocessor, self.agents[i].start_index,
                                             self.agents[i].goal_index, self.agents[i].start_angle,
                                             self.agents[i].goal_angle, self.robot_width)
                # 执行A*搜索所需要的步骤，results_path为最终规划的结果
                constraint_set = constraint_list[i]

                results_path, results_state_path, path_cost = astar_search.process(constraint_set)

                solution = {'agent': i, 'path': [], 'path_cost': path_cost}
                for s_i in results_state_path:
                    solution['path'].append(s_i)

                solution_list.append(solution)

        return solution_list


    def find_earliest_conflict(self, solution_list):
        """
        在多个 solutions 中找到最早发生的冲突。

        Args:
            mp: 地图对象，包含 edges 和节点信息。
            solution_list (list): 包含多个 solution 的列表，每个 solution 是一个字典：
                                  {'agent_idx': int, 'path': list}

        Returns:
            dict or None: 如果有冲突，则返回包含冲突信息的字典：
                          {'time': int, 'agent1': int, 'agent2': int, 'type': 'edge' or 'node', 'value': int}
                          如果没有冲突，返回 None。
        """
        num_solutions = len(solution_list)

        max_length = max(len(solution["path"]) for solution in solution_list)

        for t in range(max_length):
            # 遍历所有 solution 对
            for i in range(num_solutions):
                for j in range(i + 1, num_solutions):
                    solution1 = solution_list[i]
                    solution2 = solution_list[j]

                    # 获取两个 solution 在时间 t 的边
                    state1 = solution1["path"][t] if t < len(solution1["path"]) else None
                    state2 = solution2["path"][t] if t < len(solution2["path"]) else None

                    if state1 == -1:
                        n = 1
                        while state1 == -1:
                            state1 = solution1["path"][t + n] if t + n < len(solution1["path"]) else None
                            n += 1
                        edge1 = state1 // 2 if state1 is not None else None
                        node1 = self.mp.edges[edge1].start_end_index[0] if edge1 is not None else None
                    else:
                        edge1 = state1 // 2 if state1 is not None else None
                        node1 = self.mp.edges[edge1].start_end_index[1] if edge1 is not None else None

                    if state2 == -1:
                        n = 1
                        while state2 == -1:
                            state2 = solution2["path"][t + n] if t + n < len(solution2["path"]) else None
                            n += 1
                        edge2 = state2 // 2 if state2 is not None else None
                        node2 = self.mp.edges[edge2].start_end_index[0] if edge2 is not None else None
                    else:
                        edge2 = state2 // 2 if state2 is not None else None
                        node2 = self.mp.edges[edge2].start_end_index[1] if edge2 is not None else None

                    # 检查是否在同一时间点选择了相同的边（edge 冲突）
                    if edge1 is not None and edge2 is not None:
                        if edge1 // 2 == edge2 // 2:
                            return {
                                "time": t,
                                "agent1": solution1["agent"],
                                "agent2": solution2["agent"],
                                "type": "edge",
                                "value": (edge1, edge2)
                            }

                    # 检查是否在同一时间点到达了相同的节点（node 冲突）
                    if node1 is not None and node2 is not None:
                        if node1 == node2:
                            return {
                                "time": t,
                                "agent1": solution1["agent"],
                                "agent2": solution2["agent"],
                                "type": "node",
                                "value": (node1, node2)
                            }

                    # if edge1 is not None and edge2 is not None:
                    #     ban_pair = [[546, 740], [546, 760], [538, 738], [538, 756], [530, 736], [530, 752], [522, 748],
                    #                 [524, 750], [532, 754], [540, 758], [548, 762]]
                    #     for b in ban_pair:
                    #         if edge1 // 2 == b[0] // 2 and edge2 // 2 == b[1] // 2:
                    #             return {
                    #                 "time": t,
                    #                 "agent1": solution1["agent"],
                    #                 "agent2": solution2["agent"],
                    #                 "type": "edge",
                    #                 "value": (edge1, edge2)
                    #             }
        # 如果没有冲突，返回 None
        return None

    def count_conflicts(self, solution_list):
        """
        在多个 solutions 中统计所有发生的冲突数量。

        Args:
            solution_list (list): 包含多个 solution 的列表，每个 solution 是一个字典：
                                  {'agent_idx': int, 'path': list}

        Returns:
            int: 冲突的总数量。
        """
        num_solutions = len(solution_list)
        conflict_count = 0  # 用于统计冲突数量

        max_length = max(len(solution["path"]) for solution in solution_list)

        for t in range(max_length):
            # 遍历所有 solution 对
            for i in range(num_solutions):
                for j in range(i + 1, num_solutions):
                    solution1 = solution_list[i]
                    solution2 = solution_list[j]

                    # 获取两个 solution 在时间 t 的边
                    state1 = solution1["path"][t] if t < len(solution1["path"]) else None
                    state2 = solution2["path"][t] if t < len(solution2["path"]) else None

                    if state1 == -1:
                        n = 1
                        while state1 == -1:
                            state1 = solution1["path"][t + n] if t + n < len(solution1["path"]) else None
                            n += 1
                        edge1 = state1 // 2 if state1 is not None else None
                        node1 = self.mp.edges[edge1].start_end_index[0] if edge1 is not None else None
                    else:
                        edge1 = state1 // 2 if state1 is not None else None
                        node1 = self.mp.edges[edge1].start_end_index[1] if edge1 is not None else None

                    if state2 == -1:
                        n = 1
                        while state2 == -1:
                            state2 = solution2["path"][t + n] if t + n < len(solution2["path"]) else None
                            n += 1
                        edge2 = state2 // 2 if state2 is not None else None
                        node2 = self.mp.edges[edge2].start_end_index[0] if edge2 is not None else None
                    else:
                        edge2 = state2 // 2 if state2 is not None else None
                        node2 = self.mp.edges[edge2].start_end_index[1] if edge2 is not None else None

                    # 检查是否在同一时间点选择了相同的边（edge 冲突）
                    if edge1 is not None and edge2 is not None:
                        if edge1 // 2 == edge2 // 2:
                            c1 = self.agents[solution1["agent"]].weight
                            c2 = self.agents[solution2["agent"]].weight
                            conflict_count += (c1 + c2) / 2

                    # 检查是否在同一时间点到达了相同的节点（node 冲突）
                    if node1 is not None and node2 is not None:
                        if node1 == node2:
                            c1 = self.agents[solution1["agent"]].weight
                            c2 = self.agents[solution2["agent"]].weight
                            conflict_count += (c1 + c2) / 2

        # 返回冲突的总数量
        return conflict_count

    def is_conflict(self, solution1, solution2):
        # 获取两个路径的长度
        len1 = len(solution1["path"])
        len2 = len(solution2["path"])

        # 遍历时间点，检查是否有冲突
        for t in range(max(len1, len2)):
            # 获取两个 solution 在时间 t 的边和方向
            edge1 = solution1["path"][t] if t < len1 else None
            edge2 = solution2["path"][t] if t < len2 else None

            node1 = self.mp.edges[edge1].start_end_index[1]
            node2 = self.mp.edges[edge2].start_end_index[1]

            # 检查是否在同一时间点选择了相同的边
            if edge1 is not None and edge2 is not None:
                if edge1 // 2 == edge2 // 2:
                    return True  # 冲突发生

            # 检查是否在同一时间点选择了相同的节点
            if node1 is not None and node2 is not None:
                if node1 == node2:
                    return True

    def print_result(self):
        for i, solution in enumerate(self.solution_list):
            self.result_path[i] = []
            self.result_direction[i] = []
            path_state = solution['path']
            for state in path_state:
                edge = state // 2
                direction = "forward" if state % 2 == 0 else "reverse"
                self.result_path[i].append(edge)
                self.result_direction[i].append(direction)
        print()
        for i, path in enumerate(self.result_path):
            print("path for agent " + str(i))
            print(path)

    def plot_result(self):
        for i, solution in enumerate(self.solution_list):
            self.result_path[i] = []
            self.result_direction[i] = []
            path_state = solution['path']
            for j, state in enumerate(path_state):
                edge = state // 2
                direction = "forward" if state % 2 == 0 else "reverse"
                if state == -1:
                    state_ = state
                    k = 1
                    while state_ == -1:
                        state_ = path_state[j+k]
                        k += 1
                        direction = "forward_stay" if state_ % 2 == 0 else "reverse_stay"
                    edge = state_ // 2
                self.result_path[i].append(edge)
                self.result_direction[i].append(direction)

        db_reader = Loadmap()
        map_preprocessor = MapPreprocessing(db_reader, self.max_steer_angle_1)
        for i in range(self.num_agents):
            db_reader.plot_map(agent=i, highlight_edges=self.result_path[i], directions=self.result_direction[i])

        # db_reader.path_animation(self.result_path, self.result_direction)
        db_reader.path_animation_point_by_point(self.result_path, self.result_direction)





if __name__ == '__main__':
    cbs = CBS()

    constraint_list = [[], []]
    solution_list = [[], []]
    ct_node = CTNode(constraint_list, solution_list)
    conflict = cbs.find_solution(ct_node.constraints)

    if conflict is not None:
        time = conflict['time']
        agent_1 = conflict['agent1']
        agent_2 = conflict['agent2']
        conflict_type = conflict['type']
        value = conflict['value']

        # 给第一个冲突的agent创造CTNode
        if conflict_type == 'edge':
            node = cbs.mp.edges[value].start_end_index[1]
            constraint_ = Constraint(node, value, time)
            constraint_list[agent_1].append(constraint_)
            CTNode(constraint_list, solution_list)

        print(cbs.find_earliest_conflict(solution_list))


