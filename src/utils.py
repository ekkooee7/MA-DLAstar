class Node:
    def __init__(self, nodename, x, y):
        self.nodename = nodename
        self.post = [x, y]


class Edge:
    def __init__(self, edge_index):
        self.edge_index = edge_index
        self.edge_points = []  # List of (x, y) points
        self.start_end_index = [None, None]  # Start and end node names


class Dictionary:
    def __init__(self):
        self.forward_cost = 1
        self.reverse_cost = 1
        self.type = 0
        self.min_radius = 1
        self.road_width = 100
        self.tangent_vector_set = [None, None]


class SuccessiveEdges:
    def __init__(self):
        self.edge_index = None
        self.post = [None, None]
        self.childrenEdge = []


class SuccessiveState:
    def __init__(self):
        self.state_index = None
        self.edge_index = None
        self.end_node_index = None
        self.distance = float('inf')
        self.g = float('inf')
        self.close = False
        self.pre = -1
        self.childrenState = []
        self.visited_time = 0

    def create_constraint(self, t):
        constraint = Constraint(node_index=self.end_node_index, edge_index=self.edge_index, t=t)
        return constraint


class SuccessiveStateTime:
    def __init__(self):
        self.state_index = None
        self.edge_index = None
        self.end_node_index = None
        self.distance = float('inf')  # f = g + h
        self.g = float('inf')
        self.close = False
        self.pre = -1
        self.childrenState = []
        self.time = None

        self.constraint = self.create_constraint()

    def create_constraint(self):
        constraint = Constraint(node_index=self.end_node_index, edge_index=self.edge_index, t=self.time)
        return constraint


class Constraint:
    """
    表示单个约束，用于定义时空地图中的节点或边的限制。
    """
    def __init__(self, node_index=None, edge_index=None, t=None):
        """
        初始化约束。

        :param node_index: 节点索引 (x, y)，表示该节点在特定时间被约束。
        :param edge_index: 边索引 ((x1, y1), (x2, y2))，表示该边在特定时间被约束。
        :param t: 时间步 t，表示约束生效的时间。
        """
        self.node_index = node_index
        self.edge_index = edge_index
        self.t = t

    def __hash__(self):
        """实现哈希函数，使 Constraint 对象可作为字典键。"""
        return hash((self.node_index, self.edge_index, self.t))

    def __eq__(self, other):
        """实现比较运算符，用于字典键的相等性检查。"""
        if not isinstance(other, Constraint):
            return False
        return (self.node_index == other.node_index and
                self.edge_index == other.edge_index and
                self.t == other.t)

    def __repr__(self):
        """返回约束的字符串表示。"""
        return f"Constraint(node={self.node_index}, edge={self.edge_index}, time={self.t})"


class ReservationTable:
    """
    表示时空地图中的预留表，用于管理节点和边的约束。
    """
    def __init__(self):
        # 使用字典存储约束，key 是 Constraint 对象，value 是布尔值表示是否预留
        self.reservations = {}

    def reserve(self, constraint):
        """
        将一个约束标记为预留。

        :param constraint: Constraint 对象，表示节点或边的约束。
        """
        if not isinstance(constraint, Constraint):
            raise ValueError("The key must be a Constraint object.")
        self.reservations[constraint] = True

    def is_reserved(self, constraint):
        """
        检查一个约束是否已被预留。

        :param constraint: Constraint 对象，表示节点或边的约束。
        :return: 如果已预留，返回 True；否则返回 False。
        """
        return self.reservations.get(constraint, False)

    def release(self, constraint):
        """
        释放一个约束的预留。

        :param constraint: Constraint 对象，表示节点或边的约束。
        """
        if constraint in self.reservations:
            del self.reservations[constraint]

    def clear(self):
        """清空所有预留。"""
        self.reservations.clear()

    def get_reservations(self):
        """
        获取所有预留的约束。

        :return: 包含所有预留的约束字典。
        """
        return self.reservations