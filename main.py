import copy
import json
import random
import time

from src.constraintTreeNode import CTNode
from src.position_direction_planning import Loadmap, MapPreprocessing, AStarSearch
from src.Space_Time_AStarSearch import STAStarSearch
from src.utils import *
from src.agent import *

import src.cbs


def find_earliest_conflict(mp, solution_list):
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

    # 遍历所有 solution 对
    for i in range(num_solutions):
        for j in range(i + 1, num_solutions):
            solution1 = solution_list[i]
            solution2 = solution_list[j]

            len1 = len(solution1["path"])
            len2 = len(solution2["path"])

            # 遍历时间点，找到最早冲突
            for t in range(max(len1, len2)):
                # 获取两个 solution 在时间 t 的边
                edge1 = solution1["path"][t] if t < len1 else None
                edge2 = solution2["path"][t] if t < len2 else None

                # 获取两个 solution 在时间 t 的目标节点
                node1 = mp.edges[edge1].start_end_index[1] if edge1 is not None else None
                node2 = mp.edges[edge2].start_end_index[1] if edge2 is not None else None

                # 检查是否在同一时间点选择了相同的边（edge 冲突）
                if edge1 is not None and edge2 is not None:
                    if edge1 // 2 == edge2 // 2:
                        return {
                            "time": t,
                            "agent1": solution1["agent"],
                            "agent2": solution2["agent"],
                            "type": "edge",
                            "value": edge1
                        }

                # 检查是否在同一时间点到达了相同的节点（node 冲突）
                if node1 is not None and node2 is not None:
                    if node1 == node2:
                        return {
                            "time": t,
                            "agent1": solution1["agent"],
                            "agent2": solution2["agent"],
                            "type": "node",
                            "value": node1
                        }

    # 如果没有冲突，返回 None
    return None


def is_conflict(mp, solution1, solution2):
    # 获取两个路径的长度
    len1 = len(solution1["path"])
    len2 = len(solution2["path"])

    # 遍历时间点，检查是否有冲突
    for t in range(max(len1, len2)):
        # 获取两个 solution 在时间 t 的边和方向
        edge1 = solution1["path"][t] if t < len1 else None
        edge2 = solution2["path"][t] if t < len2 else None

        node1 = mp.edges[edge1].start_end_index[1]
        node2 = mp.edges[edge2].start_end_index[1]

        # 检查是否在同一时间点选择了相同的边
        if edge1 is not None and edge2 is not None:
            if edge1 // 2 == edge2 // 2:
                return True  # 冲突发生

        # 检查是否在同一时间点选择了相同的节点
        if node1 is not None and node2 is not None:
            if node1 == node2:
                return True


def main():
    agents = []
    solution_list = []
    robot_width = .1  # 机器人宽度（满载和空载机器人宽度不同）
    max_steer_angle = 20  # 最大转向角, 对应文章中的alpha_max

    begin = 233  # 起始点
    end = 154  # 目标点
    begin_angle = -90  # 起始角度
    end_angle = -90  # 目标角度

    agents.append(Agent((begin, begin_angle), (end, end_angle)))
    print(agents[0])

    begin = 141  # 起始点
    end = 232  # 目标点
    begin_angle = -180  # 起始角度
    end_angle = 90  # 目标角度

    agents.append(Agent((begin, begin_angle), (end, end_angle)))
    print(agents[1])

    for i in range(2):
        # 加载地图
        db_reader = Loadmap()

        db_reader.plot_map()
        # 预处理地图，参数alpha_max
        map_preprocessor = MapPreprocessing(db_reader, max_steer_angle)
        # 进行A*搜索，参数为起始点、目标点、起始角度、目标角度和机器人宽度
        astar_search = STAStarSearch(map_preprocessor, agents[i].start_index,
                                     agents[i].goal_index, agents[i].start_angle, agents[i].goal_angle, robot_width)
        # 执行A*搜索所需要的步骤，results_path为最终规划的结果
        constraint_set = []
        if i == 1:
            constraint_set = [Constraint(144, 362, 8), Constraint(176, -1, 8)]

        results_path, results_state_path = astar_search.process(constraint_set)
        # 解析results_path中的状态
        # 如果s_i % 2 == 0, 对应机器人在边int(s_i / 2), 正向行驶;
        # 如果s_i % 2 == 1, 对应机器人在边int(s_i / 2), 倒车行驶;

        solution = {'agent': i, 'path': []}

        directions = []
        for s_i in results_state_path:
            edge = s_i // 2
            direction = "forward" if s_i % 2 == 0 else "reverse"
            # print(f"Edge: {edge}, State:{s_i}, Nodes:{astar_search.edges[edge].start_end_index}, Direction: {direction}")
            directions.append(direction)
            solution['path'].append(edge)

        # print(solution["path"])
        solution_list.append(solution)

        db_reader.plot_map(highlight_edges=results_path, directions=directions)

    db_reader = Loadmap()
    # 预处理地图，参数alpha_max
    map_preprocessor = MapPreprocessing(db_reader, max_steer_angle)

    print(solution_list)
    print(is_conflict(map_preprocessor, solution_list[0], solution_list[1]))
    print(find_earliest_conflict(map_preprocessor, solution_list))


if __name__ == "__main__":
    # main()
    # 20 21 40 41 60 61 76 77 90 91 104 105 122 123 138 139 154 155 170 171 184 185 199 198 216 217 232 233 248 249

    cbs = src.cbs.CBS()


    n = 20
    loading_area = [19, 20, 21, 39, 40, 41, 59, 60, 61, 75, 76, 77, 89, 90, 91, 103, 104, 105, 121, 122, 123, 137, 138,
                    139, 153, 154, 155, 169, 170, 171, 183, 184, 185, 197, 199, 198, 215, 216, 217, 231, 232, 233,
                    247, 248, 249]

    # loading_area = [75, 76, 77, 89, 90, 91, 103, 104, 105, 121, 122, 123, 137, 138,
    #                 139, 153, 154, 155, 169, 170, 171, 183, 184, 185, 197, 199, 198]

    # loading_area = []
    # loading_area_ = [19, 39, 59, 79, 99, 119, 135, 149, 163, 177, 191, 205, 223, 239, 255, 271, 287, 303, 319, 333, 347,
    #                 361, 375, 389, 407, 423, 439, 455, 471, 487, 503, 517, 531, 545, 559, 573]
    # for a in loading_area_:
    #     loading_area.append(a)
    #     loading_area.append(a+1)
    #     loading_area.append(a+2)

    loading_area_copy = copy.deepcopy(loading_area)
    random.shuffle(loading_area_copy)
    start_index = loading_area_copy[0:n]
    # start_index = []
    start_angle = []

    max_steer_angle = 20
    # db_reader = Loadmap()
    # map_preprocessor = MapPreprocessing(db_reader, max_steer_angle)
    # random_array = list(range(0, len(map_preprocessor.successiveStateSet)))
    # random.shuffle(random_array)
    # random_array = random_array[0:n]
    #
    # for i in random_array:
    #     s = map_preprocessor.successiveStateSet[i]
    #     edge_index = s.edge_index
    #     start = map_preprocessor.edges[edge_index].start_end_index[0]
    #     # print(s.state_index)
    #     if s.state_index % 2 == 0:
    #         v1 = map_preprocessor.edges[edge_index].edge_points[1]
    #         v2 = map_preprocessor.edges[edge_index].edge_points[0]
    #     else:
    #         v1 = map_preprocessor.edges[edge_index].edge_points[0]
    #         v2 = map_preprocessor.edges[edge_index].edge_points[1]
    #
    #     dx = v1[0] - v2[0]
    #     dy = v1[1] - v2[1]
    #     angle = np.degrees(np.arctan2(dy, dx))
    #
    #     start_index.append(start)
    #     start_angle.append(angle)

    random.shuffle(loading_area_copy)
    end_index = loading_area_copy[0:n]

    start_index.sort()
    end_index.sort()

    tasks = []
    for i in range(n):
        if random.random() < 0.5:
            end_angle = 90
        else:
            end_angle = -90
        if random.random() < 0.5:
            start_angle = 90
        else:
            start_angle = -90
        if i >= n/2:
            max_steer_angle_ = 360
        else:
            max_steer_angle_ = 20
        tasks.append([start_index[i], end_index[i], start_angle, end_angle, max_steer_angle_])

    # 打开文件以写入模式
    # with open('instance/3X5_'+str(n)+'_1.json', 'w') as file:
    #     # 使用 json.dump() 方法将列表保存为 JSON 格式到文件
    #     json.dump(tasks, file)

    # 打开文件以读取模式
    # with open('instance/3X5_'+str(n)+'_4.json', 'r') as file:
    #     # 使用 json.load() 方法从文件中读取列表
    #     tasks = json.load(file)

    with open('instance/sim_3X5_'+str(n)+'_2.json', 'w') as file:
        # 使用 json.dump() 方法将列表保存为 JSON 格式到文件
        json.dump(tasks, file)

    # tasks = [[20, 77, -90, -90, max_steer_angle], [21, 19, 90, -90, max_steer_angle]]
    # tasks = [[20, 77, -90, -90, max_steer_angle], [77, 20, 90, -90, max_steer_angle]]
    # # tasks = [[232, 249, -90, -90], [248, 232, 90, -90]]
    # tasks.append([184, 217, 90, 90])
    # tasks.append([185, 231, -90, 90])
    # tasks.append([90, 138, -90, 90])
    # tasks.append([154, 105, -90, 90])
    # tasks.append([122, 170, -90, 90])
    # tasks.append([76, 40, -90, 90])
    # tasks.append([60, 104, -90, 90])
    # tasks.append([41, 76, -90, 90])
    # tasks.append([20, 60, -90, 90])

    # tasks = [[19, 89, -90, -90, max_steer_angle], [75, 39, 90, -90, max_steer_angle],
    #          [90, 21, -90, -90, max_steer_angle], [77, 41, 90, -90, max_steer_angle]]

    t1 = time.time()
    conflict = cbs.init_nodes(tasks)

    cnt = 0
    while conflict is not None:
        cnt += 1
        print(cnt)
        conflict = cbs.create_ct_node(conflict)
        # print(len(cbs.CTNodes))

    cbs.print_result()

    t2 = time.time()
    print("time cost:", t2 - t1)
    print(cnt)


    plot_flag = True
    if plot_flag:

        cbs.plot_result()


