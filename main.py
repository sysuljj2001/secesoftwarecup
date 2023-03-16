import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree #浮点数路线存储数据结构
from celluloid import Camera #动图输出

class Pid_loc:#位置比对控制
    def __init__(self,p=1.,i=0.,d=0.,point=1.,up=1.,down=-1.):
        self.p = p
        self.i = i
        self.d = d
        self.point = point
        self.up = up
        self.down = down
        self.err = 0
        self.uperr = 0
        self.sumerr = 0

    def ini_pid(self,p,i,d):
        self.p = 1.
        self.i = 0.
        self.d = 0.

    def ini_point(self,point):
        self.point = point

    def ini_jump(self,up,down):
        self.up=up
        self.down=down

    def rebegin(self):
        self.err = 0
        self.sumerr = 0
        self.uperr = 0

    def ini_sumerr(self, sumerr):
        self.sumerr = sumerr

    def output(self,loc):
        self.err = self.point-loc
        PIDresult=self.err*self.p+self.sumerr*self.i+(self.err-self.uperr)*self.d
        if PIDresult > self.up:
            PIDresult=self.up
        elif PIDresult < self.down:
            PIDresult=self.down

        self.sumerr+=self.err
        self.uperr=self.err
        return PIDresult
"""
class Pid_add:#差异比较控制
    def __init__(self,point,up,down):
        self.p = 1
        self.i = 0
        self.d = 0
        self.point = point
        self.up = up
        self.down = down
        self.err = 0
        self.lasterr = 0
        self.longerr = 0
        self.value = 0
        self.ad = 0

    def ini_pid(self):
        self.p = 1
        self.i = 0
        self.d = 0

    def ini_point(self, point):
        self.point = point

    def ini_jump(self, up, down):
        self.up = up
        self.down = down

    def update(self):
        self.value+=self.ad
        self.longerr = self.lasterr
        self.lasterr=self.err
        if self.value > self.up:
            self.value = self.up
        elif self.value < self.down:
            self.value = self.down

    def output(self,loc):
        self.err=self.point-loc
        self.ad=self.p*(self.err-self.lasterr)+self.i*self.err+self.d*(self.err-2*self.lasterr+self.longerr)
        self.update()
        return self.value
"""
class model2control:#机器人控制模块
    def __init__(self,x,y,v,L,p,dt):#机器人x坐标，机器人y坐标,机器人航行速度，机器人前后轴，机器人偏向弧度，刻长
        self.x=x
        self.y=y
        self.v=v
        self.L=L
        self.p=p
        self.dt=dt

    def update_loc(self,a,delta):
        self.x=self.x+self.v*math.cos(self.p)*self.dt
        self.y=self.y+self.v*math.sin(self.p)*self.dt
        self.p=self.p+self.v/self.L*math.tan(delta)*self.dt
        self.v=self.v+a*self.dt

    def update_get(self):
        return self.x,self.y,self.p,self.v
    #后加输出模块适配

def closepoint(bot_loc,path):
    dists=[]
    for xy in path:
        dis = np.linalg.norm(bot_loc-xy)
        dists.append(dis)
    minpoint=np.argmin(dists)
    return minpoint

PID = Pid_loc(1.3, 0.0215, 27.5, point=0, up=np.pi / 3, down=-np.pi / 3)

def main():
    # 主函数
    path = np.zeros((1000, 2))
    path[:, 0] = np.linspace(0, 100, 1000)  # 后续需考虑路径动态添加
    path[:, 1] = 2 * np.sin(path[:, 0] / 3.0)  # sin轨迹
    findtree = KDTree(path)

    bot = model2control(0 ,0 , 6, 1, 0.5, 0.1)  # 机器人x坐标，机器人y坐标,机器人航行速度，机器人前后轴，机器人偏向弧度，刻长
    x_ = []
    y_ = []
    gif = plt.figure(1)
    camera = Camera(gif)

    for i in range(550):
        loc = np.zeros(2)
        loc[0] = bot.x#最近点数据
        loc[1] = bot.y
        dis, inc = findtree.query(loc)

        alpha = math.atan2(path[inc, 1] - loc[1], path[inc, 0] - loc[0])
        long2see = np.linalg.norm(path[inc] - loc)
        # long2see=k*bot.v+c 预瞄距离
        theta = alpha - bot.p
        err2y = -long2see * math.sin(theta)
        delta = PID.output(err2y)
        bot.update_loc(0, delta)

        x_.append(bot.x)
        y_.append(bot.y)

        plt.cla()
        plt.plot(path[:, 0], path[:, 1], '-.b', linewidth=1)
        plt.plot(x_, y_, '-r', label="trajectory")
        plt.plot(path[inc, 0], path[inc, 1], "go", label="point")
        plt.grid(True)
        plt.pause(0.001)

    plt.figure(2)
    plt.plot(path[:, 0], path[:, 1], '-.b', linewidth=1)
    plt.plot(x_, y_, 'r')
    plt.show()

if __name__=='__main__':
    main()


