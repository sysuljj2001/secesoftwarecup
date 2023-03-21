import time
import math
import matplotlib.pyplot as plt
import numpy as np

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        derivative = (error - self.last_error) / dt
        self.integral += error * dt
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class Robot:
    def __init__(self, x, y, heading, linear_speed, angular_speed, path):
        self.x = x
        self.y = y
        self.heading = heading
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.path = path
        self.current_path_index = 0
        self.pid_controller = PIDController(kp=4, ki=0.0007, kd=0.00005)

    def distance_to_goal(self):
        goal = self.path[self.current_path_index]
        dx = goal[0] - self.x
        dy = goal[1] - self.y
        return math.sqrt(dx**2 + dy**2)

    def angle_to_goal(self):
        goal = self.path[self.current_path_index]
        dx = goal[0] - self.x
        dy = goal[1] - self.y
        angle_to_goal = math.atan2(dy, dx)
        return angle_to_goal - self.heading

    def addpoint(self,point1,point2):
        self.path.append(point1)
        self.path.append(point2)

    def call(self):
        return self.current_path_index

    def update(self, dt):
        distance_to_goal = self.distance_to_goal()
        angle_to_goal = self.angle_to_goal()

        if distance_to_goal < 1:
            self.current_path_index += 1
            if self.current_path_index > len(self.path):
                return False

        linear_error = distance_to_goal
        angular_error = angle_to_goal

        linear_velocity = self.pid_controller.update(linear_error, dt)
        angular_velocity = self.pid_controller.update(angular_error, dt)
        if linear_velocity > 6: linear_velocity = 6
        elif linear_velocity < -2: linear_velocity = -2
        if angular_velocity > 3:angular_velocity = 3
        elif angular_velocity < -3:angular_velocity = -3
        self.heading += angular_velocity * dt
        self.x += math.cos(self.heading) * linear_velocity * dt
        self.y += math.sin(self.heading) * linear_velocity * dt
        print(linear_velocity)
        print(angular_velocity)
        return True
    
if __name__ == '__main__':
    path = [(45.5, 10.5),(31.5, 23.5)]
    path1 = [(31.5, 10.5), (31.5, 26.5), (31.5, 23.5), (31.5, 10.5), (27.5, 20.5),(31.5, 26.5),(35.5, 10.5),(31.5, 23.5)]
    robot = Robot(x=10, y=7, heading=0, linear_speed=0, angular_speed=0, path=path)

    x_, y_ = [], []

    while robot.update(dt=0.1):
        #print(f"x: {robot.x}, y: {robot.y}, heading: {robot.heading}, current_path: {robot.current_path_index}")
        #time.sleep(0.01)

        x_.append(robot.x)
        y_.append(robot.y)
        num=robot.call()
        if num % 2 == 0 and num != 0:
            robot.path.append(path1[num - 2])#动态更新
            robot.path.append(path1[num - 1])

        plt.cla()
        plt.scatter(np.array(path)[:, 0], np.array(path)[:, 1])
        plt.plot(x_, y_, '-r', label = 'trajectory')
        plt.grid()
        plt.pause(0.001)
    plt.show()