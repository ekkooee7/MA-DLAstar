import os
import math

def calculate_edge_length(points):
    """
    计算边的长度
    :param points: 包含所有采样点坐标的列表，格式为 [(x1, y1), (x2, y2), ...]
    :return: 边的总长度
    """
    length = 0.0
    for i in range(1, len(points)):
        x1, y1 = points[i - 1]
        x2, y2 = points[i]
        length += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  # 欧几里得距离
    return length

def read_points_from_file(file_path):
    """
    从文件中读取采样点坐标
    :param file_path: 文件路径
    :return: 包含所有采样点坐标的列表，格式为 [(x1, y1), (x2, y2), ...]
    """
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            # 去除换行符并按逗号分割
            x, y = map(float, line.strip().split(','))
            points.append((x, y))
    return points

def calculate_all_edges_lengths(edges_folder):
    """
    计算所有边的长度
    :param edges_folder: 包含所有边文件的文件夹路径
    :return: 包含所有边长度的列表
    """
    edge_lengths = []
    for edge_file in os.listdir(edges_folder):
        if edge_file.endswith('.txt'):
            file_path = os.path.join(edges_folder, edge_file)
            points = read_points_from_file(file_path)
            length = calculate_edge_length(points)
            edge_lengths.append(length)
    return edge_lengths

def save_edge_lengths_to_file(edge_lengths, output_file):
    """
    将边的长度保存到文件中
    :param edge_lengths: 包含所有边长度的列表
    :param output_file: 输出文件路径
    """
    with open(output_file, 'w') as file:
        for length in edge_lengths:
            file.write(f"{length}\n")

if __name__ == "__main__":
    edges_folder = '6x10_map/edges'  # 替换为你的edges文件夹路径
    output_file = '6x10_map/weight.txt'  # 输出的文件名

    # 计算所有边的长度
    edge_lengths = calculate_all_edges_lengths(edges_folder)

    # 将边的长度保存到文件
    save_edge_lengths_to_file(edge_lengths, output_file)

    print(f"边的长度已计算并保存到 {output_file}")