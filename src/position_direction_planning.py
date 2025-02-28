import colorsys
import math
import numpy as np
from heapq import heappush, heappop

import math
import time
import heapq
from collections import defaultdict

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from src.utils import *

import yaml

# 打开文件时指定编码为 utf-8
with open('config.yaml', 'r', encoding='utf-8') as file:
    config = yaml.safe_load(file)

folder_path = config['folder_path']
weight_path = config['weight_path']
weight_reverse = config['weight_reverse']


class Loadmap:
    def __init__(self):
        self.edges = []
        self.nodes = []
        self.dictionarys = []

        self.read_edge()
        self.read_node()
        self.dictionary_construction()

        # for i, edge in enumerate(self.edges):
        #     first_point = edge.edge_points[0]
        #     last_point = edge.edge_points[-1]
        #
        #     p1 = [[157, 200], [156, 172], [108, 159], [270, 271], [63, 106], [62, 78], [3, 65], [268, 269], [27, 82],
        #           [28, 48], [80, 126], [81, 96], [127, 176], [128, 144], [174, 220], [175, 190]]
        #
        #     for p in p1:
        #         rm_node1 = self.nodes[p[0]].post
        #         rm_node2 = self.nodes[p[1]].post
        #
        #         if (first_point[0] == rm_node1[0] and first_point[1] == rm_node1[1] and
        #                 last_point[0] == rm_node2[0] and last_point[1] == rm_node2[1]):
        #             print(p)
        #             print(edge.edge_index)
                    # self.edges.remove(edge)
                    # self.dictionarys.remove(self.dictionarys[i])

                # if (first_point[0] == rm_node2[0] and first_point[1] == rm_node2[1] and
                #         last_point[0] == rm_node1[0] and last_point[1] == rm_node1[1]):
                #     print(p)
                #     print(edge.edge_index)

                    # self.dictionarys.remove(self.dictionarys[i])

    def read_node(self):
        def is_equal(node, point):
            return node.post[0] == point[0] and node.post[1] == point[1]

        def is_point_in_nodes(nodes, point):
            return any(is_equal(node, point) for node in nodes)

        node_name = 0
        for edge in self.edges:
            first_point = edge.edge_points[0]
            last_point = edge.edge_points[-1]

            if not is_point_in_nodes(self.nodes, first_point):
                self.nodes.append(Node(node_name, first_point[0], first_point[1]))
                node_name += 1

            if not is_point_in_nodes(self.nodes, last_point):
                self.nodes.append(Node(node_name, last_point[0], last_point[1]))
                node_name += 1

            # if first_point == 200 and last_point == 157:
            #     self.edges.remove(edge)

    def read_edge(self):
        # folder_path = "resource/edges"
        index_file = 0
        index_edge = 0

        while True:
            try:
                file_name = f"{folder_path}/{index_file}.txt"
                with open(file_name, "r") as file:
                    # edge的name即index
                    edge1 = Edge(index_edge)
                    edge2 = Edge(index_edge + 1)
                    temp_points = []

                    for line in file:
                        x, y = map(float, line.strip().split(","))
                        point = (x, y)
                        edge1.edge_points.append(point)
                        temp_points.append(point)

                    self.edges.append(edge1)

                    # Reverse order for the second edge
                    for point in reversed(temp_points):
                        edge2.edge_points.append(point)

                    self.edges.append(edge2)
                    index_edge += 2
                    index_file += 1
            except FileNotFoundError:
                break

    def dictionary_construction(self):
        try:
            with open(weight_path + "/weight.txt", "r") as infile:
                for cost in map(float, infile):
                    for _ in range(2):  # Forward and reverse costs
                        dictionary = Dictionary()
                        dictionary.forward_cost = cost
                        dictionary.reverse_cost = cost * weight_reverse  # Assuming k_parameter = 1.0
                        self.dictionarys.append(dictionary)

            # Fill tangent_vector_set
            for i, edge in enumerate(self.edges):
                self.dictionarys[i].tangent_vector_set = [
                    (
                        edge.edge_points[1][0] - edge.edge_points[0][0],
                        edge.edge_points[1][1] - edge.edge_points[0][1],
                    ),
                    (
                        edge.edge_points[-1][0] - edge.edge_points[-2][0],
                        edge.edge_points[-1][1] - edge.edge_points[-2][1],
                    ),
                ]

        except FileNotFoundError:
            print("Unable to open weight file")

    def plot_map(self, agent=0, highlight_edges=None, directions=None):
        """
        绘制地图，支持高亮某一条边。

        :param highlight_edge: (int) 要高亮的边的编号 (edgename)，默认为 None。
        """
        # 设置图像大小
        plt.figure(figsize=(16, 10))  # 图像大小 (宽, 高)

        if highlight_edges is None:
            highlight_edges = []
        if directions is None:
            directions = []

        # Plot edges
        for edge in self.edges:
            x_coords = [point[0] for point in edge.edge_points]
            y_coords = [point[1] for point in edge.edge_points]

            if edge.edge_index in highlight_edges:
                # 高亮边，使用特殊颜色和更粗的线条
                pass
                # plt.plot(x_coords, y_coords, color="orange", linewidth=3, label=f"Edge {edge.edge_index} (Highlighted)")
            else:
                # 普通边
                plt.plot(x_coords, y_coords, color="black", linewidth=1.5, label=f"Edge {edge.edge_index}")

        for edge in self.edges:
            x_coords = [point[0] for point in edge.edge_points]
            y_coords = [point[1] for point in edge.edge_points]

            if edge.edge_index in highlight_edges:
                i = highlight_edges.index(edge.edge_index)
                # 高亮边，使用特殊颜色和更粗的线条
                if directions[i] == "forward":
                    plt.plot(x_coords, y_coords, color="orange", linewidth=3,
                             label=f"Edge {edge.edge_index} (Highlighted)")
                elif directions[i] == "reverse":
                    plt.plot(x_coords, y_coords, color="red", linewidth=3,
                             label=f"Edge {edge.edge_index} (Highlighted)")

        # Plot nodes
        for node in self.nodes:
            plt.scatter(node.post[0], node.post[1], color='gray', zorder=5, s=50)  # 节点大小 s=50
            # plt.text(node.post[0], node.post[1], f"{node.nodename}", fontsize=8, ha="right")

        # Add labels and grid
        plt.xlabel("X Coordinate", fontsize=14)
        plt.ylabel("Y Coordinate", fontsize=14)
        plt.title("Map Visualization for agent " + str(agent), fontsize=18)
        # plt.legend(fontsize=10, loc="upper left")  # 图例调整到左上角
        plt.grid(True)
        plt.axis("equal")
        # plt.show()

        plt.savefig('figs/agent' + str(agent)+'.png', dpi=300)

    def plot_map_base(self, ax):
        """
        绘制地图的基础部分，包括普通边和节点。
        这部分内容在动画中保持不变，仅绘制一次。

        :param ax: Matplotlib Axes 对象。
        """
        # 绘制普通边
        for edge in self.edges:
            x_coords = [point[0] for point in edge.edge_points]
            y_coords = [point[1] for point in edge.edge_points]
            ax.plot(x_coords, y_coords, color="black", linewidth=1)

        # 绘制节点
        for node in self.nodes:
            ax.scatter(node.post[0], node.post[1], color="gray", zorder=1, s=10)
            # ax.text(node.post[0], node.post[1], f"{node.nodename}",
            #         fontsize=8, ha="right")

        # 设置图形格式
        # ax.set_xlabel("X Coordinate", fontsize=16)
        # ax.set_ylabel("Y Coordinate", fontsize=16)
        # ax.set_title("", fontsize=18)
        # ax.grid(True)
        ax.set_aspect("equal")

    def plot_highlight_edges(self, highlight_edges, highlight_nodes, directions, ax, lines):
        """
        更新动画中的高亮边。

        :param highlight_edges: 高亮的边的编号列表。
        :param highlight_nodes: 高亮的节点的编号列表。
        :param directions: 高亮边的方向列表（与 highlight_edges 对应）。
        :param ax: Matplotlib Axes 对象。
        :param lines: 用于存储高亮边的绘图对象列表。
        """
        # 清空之前的高亮边
        for line in lines:
            line.remove()
        lines.clear()

        n = len(highlight_edges)
        color = [tuple(colorsys.hsv_to_rgb(i / n, 1.0, 1.0)) for i in range(n)]

        for idx, node in enumerate(highlight_nodes):
            x_coord = self.nodes[node].post[0]
            y_coord = self.nodes[node].post[1]
            line = ax.scatter(x_coord, y_coord, color=color[idx], zorder=6, s=30)
            lines.append(line)

        # 绘制新的高亮边
        for idx, edge in enumerate(highlight_edges):
            if edge == -1:
                pass
                # lines.append(None)
            else:
                x_coords = [point[0] for point in self.edges[edge].edge_points]
                y_coords = [point[1] for point in self.edges[edge].edge_points]

                if directions[idx] == "forward":
                    line, = ax.plot(x_coords, y_coords, color="orange", linewidth=4,
                                    label=f"Edge {edge} (Forward)")
                elif directions[idx] == "reverse":
                    line, = ax.plot(x_coords, y_coords, color="red", linewidth=4,
                                    label=f"Edge {edge} (Reverse)")
                else:
                    line, = ax.plot(x_coords, y_coords, color="blue", linewidth=4,
                                    label=f"Edge {edge} (Unknown Direction)")

                lines.append(line)

    def path_animation(self, result_paths, result_directions):
        """
        生成动画，逐帧高亮路径。

        :param result_paths: (list) 每一帧中需要高亮的边的编号。
        :param result_directions: (list) 每一帧中高亮边的方向（与 result_paths 对应）。
        """
        fig, ax = plt.subplots(figsize=(10, 8))

        agent_num = len(result_paths)
        path_lens = [len(path) for path in result_paths]
        frames_num = max(len(sublist) for sublist in result_paths) + 1  # 加一是为了绘制出最后一个智能体到达终点的帧

        # 补齐每个子列表长度
        for k in range(len(result_paths)):
            if len(result_paths[k]) < frames_num:
                result_paths[k].extend([result_paths[k][-1]] * (frames_num - len(result_paths[k])))

        for k in range(len(result_directions)):
            if len(result_directions[k]) < frames_num:
                result_directions[k].extend([result_directions[k][-1]] * (frames_num - len(result_directions[k])))

        # 绘制基础图层（静态部分）
        self.plot_map_base(ax)

        # 用于存储高亮边的绘图对象
        lines = []
        highlight_edges = [[] for _ in range(agent_num)]
        highlight_nodes = [[] for _ in range(agent_num)]
        directions = [[] for _ in range(agent_num)]

        def update(frame):
            # 每一帧更新高亮边
            for k in range(agent_num):
                if frame >= path_lens[k]:
                    highlight_edges[k].append(-1)
                    highlight_nodes[k].append(self.edges[result_paths[k][frame]].start_end_index[1])
                    directions[k].append(result_directions[k][frame])
                    continue

                if frame < path_lens[k] - 1:
                    if result_paths[k][frame] == result_paths[k][frame + 1]:
                        highlight_edges[k].append(-1)
                    elif result_paths[k][frame] != result_paths[k][frame + 1]:
                        highlight_edges[k].append(result_paths[k][frame])
                else:
                    highlight_edges[k].append(result_paths[k][frame])

                highlight_nodes[k].append(self.edges[result_paths[k][frame]].start_end_index[0])
                directions[k].append(result_directions[k][frame])

            self.plot_highlight_edges([sublist[-1] for sublist in highlight_edges],
                                      [sublist[-1] for sublist in highlight_nodes],
                                      [sublist[-1] for sublist in directions],
                                      ax, lines)

        t1 = time.time()
        # 创建动画（使用 blit 提升性能）
        ani = FuncAnimation(fig, update, frames=frames_num, repeat=False, interval=50, blit=False)

        plt.show()
        # 保存动画
        ani.save('path_animation.mp4', writer='ffmpeg', fps=4)
        t2 = time.time()
        print("animation generation time：", t2 - t1)

    def plot_highlight_edges_and_nodes(self, highlight_edges, highlight_nodes, directions, ax, lines, nodes, index,
                                       seg):
        """
        更新动画中的高亮边。

        :param highlight_edges: 高亮的边的编号列表。
        :param highlight_nodes: 高亮的节点的编号列表。
        :param directions: 高亮边的方向列表（与 highlight_edges 对应）。
        :param ax: Matplotlib Axes 对象。
        :param lines: 用于存储高亮边的绘图对象列表。
        """
        # 清空之前的高亮边
        for line in lines:
            line.remove()
        lines.clear()

        for node in nodes:
            node.remove()
        nodes.clear()

        n = len(highlight_edges)
        color = [tuple(colorsys.hsv_to_rgb(i / n, 1.0, 1.0)) for i in range(n)]

        # for idx, node in enumerate(highlight_nodes):
        #     x_coord = self.nodes[node].post[0]
        #     y_coord = self.nodes[node].post[1]
        # line = ax.scatter(x_coord, y_coord, color=color[idx], zorder=6, s=60)
        # lines.append(line)

        # 绘制新的高亮边
        for idx, edge in enumerate(highlight_edges):
            if edge == -1:
                pass
                # lines.append(None)
            else:
                x_coords = [point[0] for point in self.edges[edge].edge_points]
                y_coords = [point[1] for point in self.edges[edge].edge_points]

                # if directions[idx] == "forward":
                #     line, = ax.plot(x_coords, y_coords, color="orange", linewidth=2.5,
                #                     label=f"Edge {edge} (Forward)")
                # elif directions[idx] == "reverse":
                #     line, = ax.plot(x_coords, y_coords, color="red", linewidth=2.5,
                #                     label=f"Edge {edge} (Reverse)")
                # else:
                #     line, = ax.plot(x_coords, y_coords, color="blue", linewidth=2.5,
                #                     label=f"Edge {edge} (Unknown Direction)")

                # lines.append(line)
        for idx, edge in enumerate(highlight_edges):

            x_coords = [point[0] for point in self.edges[edge].edge_points]
            y_coords = [point[1] for point in self.edges[edge].edge_points]
            if directions[idx] == "forward_stay":
                index_ = int(1)
                u = x_coords[index_ + 1] - x_coords[index_]
                v = y_coords[index_ + 1] - y_coords[index_]
            elif directions[idx] == "reverse_stay":
                index_ = int(1)
                u = x_coords[index_] - x_coords[index_ + 1]
                v = y_coords[index_] - y_coords[index_ + 1]
            elif directions[idx] == "forward_end":
                index_ = int(98)
                u = x_coords[index_ + 1] - x_coords[index_]
                v = y_coords[index_ + 1] - y_coords[index_]
            elif directions[idx] == "reverse_end":
                index_ = int(98)
                u = x_coords[index_] - x_coords[index_ + 1]
                v = y_coords[index_] - y_coords[index_ + 1]
            else:
                index_ = int(100 / seg * index + 1)
                if directions[idx] == "forward":
                    u = x_coords[index_ + 1] - x_coords[index_]
                    v = y_coords[index_ + 1] - y_coords[index_]
                else:
                    u = x_coords[index_] - x_coords[index_ + 1]
                    v = y_coords[index_] - y_coords[index_ + 1]

            vec = [u, v]
            norm = np.linalg.norm(vec)
            vec = vec / norm
            a = ax.arrow(x_coords[index_] - vec[0] * 3, y_coords[index_] - vec[1] * 3, vec[0] * 10, vec[1] * 10,
                         head_width=18, head_length=20, fc=color[idx], ec='black', zorder=5, lw=0.5)

            # lines.append(line)
            # nodes.append(node)
            # nodes.append(q)
            nodes.append(a)

    def path_animation_point_by_point(self, result_paths, result_directions):
        """
        :param result_paths: (list) 每一帧中需要高亮的边的编号。
        :param result_directions: (list) 每一帧中高亮边的方向（与 result_paths 对应）。
        """
        fig, ax = plt.subplots(figsize=(16, 12))

        agent_num = len(result_paths)
        path_lens = [len(path) for path in result_paths]
        frames_num = (max(len(sublist) for sublist in result_paths) + 1)  # 加一是为了绘制出最后一个智能体到达终点的帧

        # 补齐每个子列表长度
        for k in range(len(result_paths)):
            if len(result_paths[k]) < frames_num:
                result_paths[k].extend([result_paths[k][-1]] * (frames_num - len(result_paths[k])))

        for k in range(len(result_directions)):
            if len(result_directions[k]) < frames_num:
                result_directions[k].extend([result_directions[k][-1]] * (frames_num - len(result_directions[k])))

        # 绘制基础图层（静态部分）
        self.plot_map_base(ax)

        # 用于存储高亮边的绘图对象
        lines = []
        nodes = []
        highlight_edges = [[] for _ in range(agent_num)]
        highlight_nodes = [[] for _ in range(agent_num)]
        directions = [[] for _ in range(agent_num)]

        seg = 5

        def update(frame):
            # 每一帧更新高亮边

            n = frame % seg
            frame = math.floor(frame / seg)
            for k in range(agent_num):
                if frame >= path_lens[k]:
                    highlight_edges[k].append(result_paths[k][path_lens[k] - 1])
                    highlight_nodes[k].append(self.edges[result_paths[k][frame]].start_end_index[1])
                    if result_directions[k][path_lens[k] - 1] == "forward":
                        directions[k].append("forward_end")
                    elif result_directions[k][path_lens[k] - 1] == "reverse":
                        directions[k].append("reverse_end")
                    continue

                if frame < path_lens[k] - 1:
                    if result_paths[k][frame] == result_paths[k][frame + 1]:
                        highlight_edges[k].append(result_paths[k][frame])
                    elif result_paths[k][frame] != result_paths[k][frame + 1]:
                        highlight_edges[k].append(result_paths[k][frame])
                else:
                    highlight_edges[k].append(result_paths[k][frame])

                highlight_nodes[k].append(self.edges[result_paths[k][frame]].start_end_index[0])
                directions[k].append(result_directions[k][frame])


            self.plot_highlight_edges_and_nodes([sublist[-1] for sublist in highlight_edges],
                                                [sublist[-1] for sublist in highlight_nodes],
                                                [sublist[-1] for sublist in directions],
                                                ax, lines, nodes, n, seg)

        t1 = time.time()
        # 创建动画（使用 blit 提升性能）
        ani = FuncAnimation(fig, update, frames=frames_num * seg, repeat=False, interval=5, blit=False)

        plt.show()
        # 保存动画
        ani.save('path_animation.mp4', writer='ffmpeg', fps=10)
        t2 = time.time()
        print("animation generation time：", t2 - t1)


class MapPreprocessing:
    def __init__(self, database_reader, max_steer_angle):
        self.edges = database_reader.edges
        self.nodes = database_reader.nodes

        self.dictionarys = database_reader.dictionarys

        self.r_min = 0.5

        self.edgeNum = len(self.edges)
        self.nodeNum = len(self.nodes)
        self.max_steer_angle = max_steer_angle

        self.successiveEdgeSet = [SuccessiveEdges() for _ in range(self.edgeNum)]
        self.successiveStateSet = [SuccessiveState() for _ in range(2 * self.edgeNum)]

        self.generate_successive_edge_set()
        self.preprocessing()

    def generate_successive_edge_set(self):
        for i, edge in enumerate(self.edges):
            for node in self.nodes:
                # record the start node of the edge
                if (edge.edge_points[0][0], edge.edge_points[0][1]) == (node.post[0], node.post[1]):
                    edge.start_end_index[0] = node.nodename

                # record the end node of the edge
                if (edge.edge_points[-1][0], edge.edge_points[-1][1]) == (node.post[0], node.post[1]):
                    edge.start_end_index[1] = node.nodename

            # start = edge.start_end_vector[0]
            # end = edge.start_end_vector[1]
        for i, edge in enumerate(self.edges):
            self.successiveEdgeSet[i].post[0] = edge.start_end_index[0]
            self.successiveEdgeSet[i].post[1] = edge.start_end_index[1]
            self.successiveEdgeSet[i].edge_index = edge.edge_index

            cnt = 0
            for j, other_edge in enumerate(self.edges):
                if edge.start_end_index[1] == other_edge.start_end_index[0]:
                    self.successiveEdgeSet[i].childrenEdge.append(other_edge.edge_index)
                    cnt += 1

            # cn = cnt
            # print(self.successiveEdgeSet[i].childrenEdge)

    def vector_angle(self, vec1, vec2):
        dot = vec1[0] * vec2[0] + vec1[1] * vec2[1]
        # 计算两个向量的模（长度）
        mag1 = math.sqrt(vec1[0] ** 2 + vec1[1] ** 2)
        mag2 = math.sqrt(vec2[0] ** 2 + vec2[1] ** 2)
        # 防止除以零，若有零向量则直接返回 0 度
        if mag1 == 0 or mag2 == 0:
            return 0
        # 计算 cos(theta)，限制范围在 [-1, 1] 以避免浮点数精度问题
        cos_theta = max(-1, min(1, dot / (mag1 * mag2)))
        # 使用 math.acos 计算角度（弧度），并转换为角度
        angle_degrees = math.degrees(math.acos(cos_theta))
        return angle_degrees

    def cal_distance(self, x, y):
        # 计算欧式距离的平方
        temp = (x.post[0] - y.post[0]) ** 2 + (x.post[1] - y.post[1]) ** 2
        # 返回平方根（欧式距离）
        return math.sqrt(temp)

    def preprocessing(self):
        # 遍历所有的边
        for i in range(self.edgeNum):
            # 初始化每条边下一状态的正向和反向两种类型状态
            self.successiveStateSet[2 * i].state_index = 2 * i
            self.successiveStateSet[2 * i].edge_index = i
            self.successiveStateSet[2 * i].distance = float('inf')
            self.successiveStateSet[2 * i].close = False
            self.successiveStateSet[2 * i].pre = -1

            self.successiveStateSet[2 * i + 1].state_index = 2 * i + 1
            self.successiveStateSet[2 * i + 1].edge_index = i
            self.successiveStateSet[2 * i + 1].distance = float('inf')
            self.successiveStateSet[2 * i + 1].close = False
            self.successiveStateSet[2 * i + 1].pre = -1

            # 获取当前边的终点切线向量
            tangent_vector_i_1 = self.dictionarys[i].tangent_vector_set[1]

            # 遍历当前边的所有后继边
            for r in self.successiveEdgeSet[i].childrenEdge:
                r_j = self.dictionarys[r].min_radius
                tangent_vector_j_0 = self.dictionarys[r].tangent_vector_set[0]
                theta_ij = self.vector_angle(tangent_vector_i_1, tangent_vector_j_0)
                # 检查夹角和曲率半径限制
                if theta_ij <= self.max_steer_angle and r_j >= self.r_min:
                    # 正向衔接正向
                    self.successiveStateSet[2 * i].childrenState.append(2 * r)
                    # 反向衔接反向
                    self.successiveStateSet[2 * i + 1].childrenState.append(2 * r + 1)
                if 180 - theta_ij <= self.max_steer_angle and r_j >= self.r_min:
                    # 正向衔接反向
                    self.successiveStateSet[2 * i].childrenState.append(2 * r + 1)
                    # 反向衔接正向
                    self.successiveStateSet[2 * i + 1].childrenState.append(2 * r)


class AStarSearch:
    def __init__(self, database_reader, preprocessed_map, A_begin, A_end, A_beginAngle, A_endAngle, A_robot_width,
                 start_state=None):
        """
        Python 等价构造函数
        """
        # 初始化外部对象
        self.database_reader = database_reader
        self.preprocessed_map = preprocessed_map

        # 初始化地图数据
        # self.time_start_mapload = databaseReader.getTime_start_mapload()
        self.max_steer_angle = preprocessed_map.max_steer_angle
        self.edges = database_reader.edges
        self.nodes = database_reader.nodes
        self.dictionarys = database_reader.dictionarys
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
        self.start_state = start_state

        self.end_stage = None
        self.final_path_distance = None

        # 计算起点和终点的方向向量
        self.phi_start = (math.cos(math.radians(self.beginAngle)), math.sin(math.radians(self.beginAngle)))
        self.phi_end = (math.cos(math.radians(self.endAngle)), math.sin(math.radians(self.endAngle)))

        # 初始化路径结果
        self.resultsEdgePath = []
        self.resultsStatePath = []

    def searching(self):
        """
        A* 搜索算法实现
        """
        # 优先队列（基于 heapq），存储 (代价, 状态编号)
        pq = []
        heapq.heapify(pq)

        if self.start_state is not None:
            # 直接输入state作为初始的state
            edge_index = self.successiveStateSet[self.start_state].edge_index
            distance = (self.dictionarys[edge_index].reverse_cost
                        + self.preprocessed_map.cal_distance(
                        self.nodes[self.edges[edge_index].start_end_index[1]], self.nodes[self.end]))
            self.successiveStateSet[self.start_state].distance = distance
            self.successiveStateSet[self.start_state].g = 0
            heapq.heappush(pq, (distance, self.start_state))
        else:
            # 输入位置和车辆朝向，遍历所有边，初始化起点的可达state
            for r in self.edges:
                if r.start_end_index[0] == self.begin:
                    t_start = self.dictionarys[r.edge_index].tangent_vector_set[0]
                    theta_start = self.preprocessed_map.vector_angle(self.phi_start, t_start)

                    # 正向可达状态
                    if (
                            theta_start <= self.max_steer_angle
                            and self.dictionarys[r.edge_index].min_radius >= self.preprocessed_map.r_min
                            and self.dictionarys[r.edge_index].road_width > self.robot_width
                    ):
                        distance = (self.dictionarys[r.edge_index].forward_cost
                                    + self.preprocessed_map.cal_distance(
                                    self.nodes[self.edges[r.edge_index].start_end_index[1]], self.nodes[self.end]))
                        self.successiveStateSet[2 * r.edge_index].distance = distance
                        self.successiveStateSet[2 * r.edge_index].g = 0
                        heapq.heappush(pq, (distance, 2 * r.edge_index))

                    # 反向可达状态
                    if (
                            180 - theta_start <= self.max_steer_angle
                            and self.dictionarys[r.edge_index].min_radius >= self.preprocessed_map.r_min
                            and self.dictionarys[r.edge_index].road_width > self.robot_width
                    ):
                        distance = (self.dictionarys[r.edge_index].reverse_cost
                                    + self.preprocessed_map.cal_distance(
                                    self.nodes[self.edges[r.edge_index].start_end_index[1]], self.nodes[self.end]))
                        self.successiveStateSet[2 * r.edge_index + 1].distance = distance
                        self.successiveStateSet[2 * r.edge_index + 1].g = 0
                        heapq.heappush(pq, (distance, 2 * r.edge_index + 1))
        count = 0

        # 主循环
        while pq:
            count += 1

            # 取出优先队列中代价最小的状态
            _, u = heapq.heappop(pq)
            edge_index = self.successiveStateSet[u].edge_index
            # self.database_reader.plot_map(highlight_edges=[edge_index])

            # 如果状态已在闭集中，则跳过
            if self.successiveStateSet[u].close:
                continue
            self.successiveStateSet[u].close = True

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
                    self.final_path_distance = self.successiveStateSet[u].distance
                    self.end_stage = u
                    break

            # 更新子节点
            for v in self.successiveStateSet[u].childrenState:
                u_edge_index = self.successiveStateSet[u].edge_index
                v_edge_index = self.successiveStateSet[v].edge_index
                # self.database_reader.plot_map(highlight_edges=[v_edge_index])

                # 计算 g(u)
                g_u = self.successiveStateSet[u].distance - self.preprocessed_map.cal_distance(
                    self.nodes[self.edges[u_edge_index].start_end_index[1]], self.nodes[self.end]
                )

                # 正向子节点
                if (
                        v % 2 == 0
                        and self.successiveStateSet[v].distance
                        > g_u
                        + self.dictionarys[v_edge_index].forward_cost
                        + self.preprocessed_map.cal_distance(
                    self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                )
                        and self.dictionarys[v_edge_index].road_width > self.robot_width
                ):
                    self.successiveStateSet[v].distance = (
                            g_u
                            + self.dictionarys[v_edge_index].forward_cost
                            + self.preprocessed_map.cal_distance(
                        self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                    )
                    )
                    self.successiveStateSet[v].g = g_u
                    self.successiveStateSet[v].pre = u
                    heapq.heappush(pq, (self.successiveStateSet[v].distance, v))

                # 反向子节点
                if (
                        v % 2 == 1
                        and self.successiveStateSet[v].distance
                        > g_u
                        + self.dictionarys[v_edge_index].reverse_cost
                        + self.preprocessed_map.cal_distance(
                    self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                )
                        and self.dictionarys[v_edge_index].road_width > self.robot_width
                ):
                    self.successiveStateSet[v].distance = (
                            g_u
                            + self.dictionarys[v_edge_index].reverse_cost
                            + self.preprocessed_map.cal_distance(
                        self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                    )
                    )
                    self.successiveStateSet[v].g = g_u
                    self.successiveStateSet[v].pre = u
                    heapq.heappush(pq, (self.successiveStateSet[v].distance, v))

    def process(self):
        """
        回溯路径
        """
        self.searching()

        if self.end_stage is None:
            print("No path found")
            return []

        print("path length: ", self.final_path_distance)

        # 回溯路径
        self.resultsStatePath.insert(0, self.end_stage)
        temp_pre = self.successiveStateSet[self.end_stage].pre

        while temp_pre != -1:
            self.resultsStatePath.insert(0, temp_pre)
            temp_pre = self.successiveStateSet[temp_pre].pre

        for state in self.resultsStatePath:
            self.resultsEdgePath.append(self.successiveStateSet[state].edge_index)

        return self.resultsEdgePath, self.resultsStatePath

    def return_min_dis(self):
        """
        返回此次路径规划的每个State到终点的最短路径，以减少查询次数
        """
        self.searching()

        state_to_goal_dis = []

        if self.end_stage is None:
            print("No path found")
            return []

        state_to_goal_dis.append((self.end_stage, self.final_path_distance - self.successiveStateSet[self.end_stage].g))
        # print("path length: ", self.final_path_distance)

        # 回溯路径
        self.resultsStatePath.insert(0, self.end_stage)
        temp_pre = self.successiveStateSet[self.end_stage].pre

        while temp_pre != -1:
            # print(temp_pre)
            self.resultsStatePath.insert(0, temp_pre)
            ####
            g = self.successiveStateSet[temp_pre].g
            edge_index = self.successiveStateSet[temp_pre].edge_index
            start = self.edges[edge_index].start_end_index[0]
            # print(self.successiveStateSet[temp_pre].state_index)
            if self.successiveStateSet[temp_pre].state_index % 2 == 0:
                v1 = self.edges[edge_index].edge_points[1]
                v2 = self.edges[edge_index].edge_points[0]
                g = g
            else:
                v1 = self.edges[edge_index].edge_points[0]
                v2 = self.edges[edge_index].edge_points[1]
                g = g
            dx = v1[0] - v2[0]
            dy = v1[1] - v2[1]
            angle = np.degrees(np.arctan2(dy, dx))
            # print(start, angle)
            # print("in process path length", self.final_path_distance - g)
            state_to_goal_dis.append((temp_pre, self.final_path_distance - g))

            ####
            temp_pre = self.successiveStateSet[temp_pre].pre

        return state_to_goal_dis

    def search_all_node(self, begin, phi_start):
        """
        A* 搜索算法实现
        """
        # 优先队列（基于 heapq），存储 (代价, 状态编号)
        pq = []
        heapq.heapify(pq)

        # 遍历所有边，初始化起点的可达状态
        for r in self.edges:
            if r.start_end_index[0] == begin:
                t_start = self.dictionarys[r.edge_index].tangent_vector_set[0]
                theta_start = self.preprocessed_map.vector_angle(phi_start, t_start)

                # 正向可达状态
                if (
                        theta_start <= self.max_steer_angle
                        and self.dictionarys[r.edge_index].min_radius >= self.preprocessed_map.r_min
                        and self.dictionarys[r.edge_index].road_width > self.robot_width
                ):
                    distance = (self.dictionarys[r.edge_index].forward_cost
                                + self.preprocessed_map.cal_distance(
                                self.nodes[self.edges[r.edge_index].start_end_index[1]], self.nodes[self.end]))
                    self.successiveStateSet[2 * r.edge_index].distance = distance
                    heapq.heappush(pq, (distance, 2 * r.edge_index))

                # 反向可达状态
                if (
                        180 - theta_start <= self.max_steer_angle
                        and self.dictionarys[r.edge_index].min_radius >= self.preprocessed_map.r_min
                        and self.dictionarys[r.edge_index].road_width > self.robot_width
                ):
                    distance = (self.dictionarys[r.edge_index].reverse_cost
                                + self.preprocessed_map.cal_distance(
                                self.nodes[self.edges[r.edge_index].start_end_index[1]], self.nodes[self.end]))
                    self.successiveStateSet[2 * r.edge_index + 1].distance = distance
                    heapq.heappush(pq, (distance, 2 * r.edge_index + 1))
        count = 0

        # 主循环
        while pq:
            count += 1

            # 取出优先队列中代价最小的状态
            d, u = heapq.heappop(pq)

            # 如果状态已在闭集中，则跳过
            if self.successiveStateSet[u].close:
                continue
            self.successiveStateSet[u].close = True

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
                    return d
                    break

            # 更新子节点
            for v in self.successiveStateSet[u].childrenState:
                u_edge_index = self.successiveStateSet[u].edge_index
                v_edge_index = self.successiveStateSet[v].edge_index
                # self.database_reader.plot_map(highlight_edges=[v_edge_index])

                # 计算 g(u)
                g_u = self.successiveStateSet[u].distance - self.preprocessed_map.cal_distance(
                    self.nodes[self.edges[u_edge_index].start_end_index[1]], self.nodes[self.end]
                )

                # 正向子节点
                if (
                        v % 2 == 0
                        and self.successiveStateSet[v].distance
                        > g_u
                        + self.dictionarys[v_edge_index].forward_cost
                        + self.preprocessed_map.cal_distance(
                    self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                )
                        and self.dictionarys[v_edge_index].road_width > self.robot_width
                ):
                    self.successiveStateSet[v].distance = (
                            g_u
                            + self.dictionarys[v_edge_index].forward_cost
                            + self.preprocessed_map.cal_distance(
                        self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                    )
                    )
                    self.successiveStateSet[v].pre = u
                    heapq.heappush(pq, (self.successiveStateSet[v].distance, v))

                # 反向子节点
                if (
                        v % 2 == 1
                        and self.successiveStateSet[v].distance
                        > g_u
                        + self.dictionarys[v_edge_index].reverse_cost
                        + self.preprocessed_map.cal_distance(
                    self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                )
                        and self.dictionarys[v_edge_index].road_width > self.robot_width
                ):
                    self.successiveStateSet[v].distance = (
                            g_u
                            + self.dictionarys[v_edge_index].reverse_cost
                            + self.preprocessed_map.cal_distance(
                        self.nodes[self.edges[v_edge_index].start_end_index[1]], self.nodes[self.end]
                    )
                    )
                    self.successiveStateSet[v].pre = u
                    heapq.heappush(pq, (self.successiveStateSet[v].distance, v))
