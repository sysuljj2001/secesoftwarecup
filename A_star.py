import numpy as np

direction = [[1, 1], [1, 0], [1, -1], [0, -1], [-1, -1], [-1, 0], [-1, 1]]

def get_pal(x, y):
    # 从方向获取偏航角
    return np.arctan(y / x)

def get_rad(ang):
    return np.pi / 180 * ang
    
def get_ang(rad):
    return 180 / np.pi * rad

class node():
    def __init__(self, x, y, pal, fa) -> None:
        self.x = x
        self.y = y
        self.pal = pal
        self.pre = fa
        self.g = None

class Map():
    def __init__(self, h, w) -> None:
        # 地图设置
        self.m = np.zeros((h, w))
        # 最大偏航角 30 度
        self.max_dpal = get_rad(30)
    


    def set_block(self, x, y):
        '''设置障碍物
        '''
        pass

def A_star(x, y, pal, u, v, map: Map):
    '''
    (x, y): 起点
    pal: 初始朝向
    (u, v): 终点
    '''
    vis = {}
    start = node(x, y, pal, -1)
    node_que = [start]
    while(len(node_que)):
        nw_node = None
        for no in node_que:
            pass
        if(vis.get(nw_node)): continue
        vis[nw_node] = 1
        for d in direction:
            # 根据最大偏航角计算可移动点 h
            if(vis.get(nx_node)): continue
            nx_node = node(x + d[0], y + d[1], nw_node.pal, nw_node)
            if map.m[nx_node.x, nx_node.y] != 0:
                continue
            dang = np.abs(get_ang(np.arctan(d[1] / d[0])) - get_ang(nw_node.pal))
            if(dang < map.max_dpal): 
                nx_node.pal = np.arctan(d[1] / d[0])
            else: nx_node.pal = nw_node.pal + map.max_dpal * np.sign(np.arctan(d[0] / d[1]) - nw_node.pal)
            h = np.exp(dang) + np.abs(nx_node.x - u) + np.abs(nx_node.y - v)
            f = nw_node.f + int(np.sqrt(d[0] ** 2 - d[1] ** 2) * 10)
            if(h + f < nx_node.g or nx_node.g is None): 
                nx_node.g = h + f
                nx_node.pre = nw_node
            node_que.append(nx_node)
        