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
        self.pid_controller = PIDController(kp=0.1, ki=1.2, kd=0.5)

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

    def update(self, dt):
        distance_to_goal = self.distance_to_goal()
        angle_to_goal = self.angle_to_goal()

        if distance_to_goal < 0.1:
            self.current_path_index += 1
            if self.current_path_index >= len(self.path):
                return False

        linear_error = distance_to_goal
        angular_error = angle_to_goal

        linear_velocity = self.pid_controller.update(linear_error, dt)
        angular_velocity = self.pid_controller.update(angular_error, dt)
        if linear_velocity > 6: linear_velocity = 2
        if angular_velocity > 6: angular_velocity = 2

        self.heading += angular_velocity * dt
        self.x += math.cos(self.heading) * linear_velocity * dt
        self.y += math.sin(self.heading) * linear_velocity * dt

        return True
    
if __name__ == '__main__':
    path = [(0, 0), (1, 1), (2, 0), (3, 1), (4, 2), (5, 3), (6, 4), (7, 5), (6, 6), (7, 7)]

    robot = Robot(x=0, y=0, heading=0, linear_speed=1, angular_speed=1, path=path)

    x_, y_ = [], []

    while robot.update(dt=0.001):
        print(f"x: {robot.x}, y: {robot.y}, heading: {robot.heading}, current_path: {robot.current_path_index}")
        time.sleep(0.001)

        x_.append(robot.x)
        y_.append(robot.y)

        plt.cla()
        plt.scatter(np.array(path)[:, 0], np.array(path)[:, 1])
        plt.plot(x_, y_, '-r', label = 'trajectory')
        plt.grid()
        plt.pause(0.001)