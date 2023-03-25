import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Robot:
    def __init__(self, start_position, end_position, initial_velocity, max_speed, max_angular_speed, radius):
        self.position = np.array(start_position, dtype=np.float64)
        self.end_position = np.array(end_position, dtype=np.float64)
        self.velocity = np.array(initial_velocity, dtype=np.float64)
        self.max_speed = max_speed
        self.max_angular_speed = max_angular_speed
        self.radius = radius

def normalize(vector):
    return vector / np.linalg.norm(vector)

def compute_new_velocity(robot, neighbors, dt, goal_velocity):
    # 沿目标方向的速度可能会引起碰撞的权重
    collision_weight = 100

    candidate_velocities = [goal_velocity + dt * np.array([np.cos(angle), np.sin(angle)]) for angle in np.linspace(0, 2 * np.pi, 100)]

    min_cost = float('inf')
    best_velocity = None

    for candidate_velocity in candidate_velocities:
        cost = np.linalg.norm(candidate_velocity - goal_velocity)
        
        # 如果候选速度导致碰撞，增加碰撞权重
        for neighbor in neighbors:
            if will_collide(robot, neighbor, candidate_velocity, dt):
                cost += collision_weight * np.linalg.norm(candidate_velocity - neighbor.velocity)

        if cost < min_cost:
            min_cost = cost
            best_velocity = candidate_velocity

    return best_velocity

def will_collide(robot1, robot2, candidate_velocity, dt):
    relative_position = robot2.position - robot1.position
    relative_velocity = candidate_velocity - robot2.velocity
    distance = np.linalg.norm(relative_position)

    if distance <= robot1.radius + robot2.radius:
        return True

    t = -np.dot(relative_position, relative_velocity) / np.dot(relative_velocity, relative_velocity)

    if t < 0 or t > dt:
        return False

    nearest_approach = relative_position + t * relative_velocity

    return np.linalg.norm(nearest_approach) <= robot1.radius + robot2.radius

def rvo_multi_robot_path_planning(robots, time_step, max_iterations):
    trajectories = {robot: [robot.position] for robot in robots}

    for _ in range(max_iterations):
        for robot in robots:
            # 计算目标方向的期望速度
            goal_direction = normalize(robot.end_position - robot.position) * robot.max_speed

            neighbors = [other_robot for other_robot in robots if other_robot != robot and np.linalg.norm(other_robot.position - robot.position) < robot.radius * 5]
            new_velocity = compute_new_velocity(robot, neighbors, time_step, goal_direction)
            robot.position = robot.position + time_step * new_velocity
            trajectories[robot].append(robot.position)

            if np.linalg.norm(robot.position - robot.end_position) < robot.radius:
                robots.remove(robot)

        if not robots:
            break

    return trajectories

def visualize_trajectories(trajectories):
    fig, ax = plt.subplots()
    ax.set_xlim(-5, 15)
    ax.set_ylim(-5, 15)

    colors = ['blue', 'red', 'green', 'orange']

    def update(frame_number):
        ax.clear()
        ax.set_xlim(-5, 15)
        ax.set_ylim(-5, 15)

        for i, (robot, traj) in enumerate(trajectories.items()):
            position = traj[frame_number]
            ax.scatter(*position, c=colors[i], marker='o')
            ax.text(position[0] + 0.25, position[1], f"R{i + 1}")

    anim = FuncAnimation(fig, update, frames=len(trajectories[next(iter(trajectories))]), interval=200, repeat=False)
    plt.show()

def main():
    time_step = 0.2
    max_iterations = 200

    robots = [
        Robot((0, 0), (10, 10), (0, 0), 2, 0.5),
        Robot((10, 0), (0, 10), (0, 0), 2, 0.5),
        Robot((0, 10), (10, 0), (0, 0), 2, 0.5),
        Robot((10, 10), (0, 0), (0, 0), 2, 0.5),
    ]

    trajectories = rvo_multi_robot_path_planning(robots, time_step, max_iterations)
    visualize_trajectories(trajectories)

if __name__ == "__main__":
    main()
