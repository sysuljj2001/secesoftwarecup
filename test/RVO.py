from numpy import ceil, floor, sqrt
from numpy import sin, cos, tan, arctan2, arcsin
import numpy

# DEFINE CONSTANT
PI = 3.141592653589793


def distance(p1, p2):
    """To calculate liner distance in the map
        p1[0]: the x coordination of position 1;    p1[1]:the y coordination of position 1
        p2[0]: the x coordination of position 2;    p2[1]:the y coordination of position 2
        distance: Euclidean distance between p1 and p2 in 2D Space"""

    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) + 0.001


def RVO_update(X, desire_V, V_current, ws_model):
    """Give the desire velocity,current velocity and workstation model.Return the best velocity of robots."""
    Robot_Radius = ws_model['robot_radius'] + 0.1  # Set a virtual wall around robot
    V_opt = list(V_current)
    for i in range(len(X)):
        vA = [V_current[i][0], V_current[i][1]]
        pA = [X[i][0], X[i][1]]
        RVO_BA_all = []
        for j in range(len(X)):
            if i != j:
                vB = [V_current[j][0], V_current[j][1]]
                pB = [X[j][0], X[j][1]]
                transl_vB_vA = [pA[0] + 0.5 * (vB[0] + vA[0]), pA[1] + 0.5 * (vB[1] + vA[1])]
                dist_BA = distance(pA, pB)
                theta_BA = arctan2(pB[1] - pA[1], pB[0] - pA[0])
                if 2 * Robot_Radius > dist_BA:
                    dist_BA = 2 * Robot_Radius
                theta_BAort = arcsin(2 * Robot_Radius / dist_BA)
                theta_ort_left = theta_BA + theta_BAort
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                theta_ort_right = theta_BA - theta_BAort
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2 * Robot_Radius]
                RVO_BA_all.append(RVO_BA)
        vA_post = intersect(pA, desire_V[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt


def intersect(pA, vA, RVO_BA_all):
    norm_v = distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []
    for theta in numpy.arange(0, 2 * PI, 0.1):
        for rad in numpy.arange(0.02, norm_v + 0.02, norm_v / 5.0):
            new_v = [rad * cos(theta), rad * sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
                theta_dif = arctan2(dif[1], dif[0])
                theta_right = arctan2(right[1], right[0])
                theta_left = arctan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
        theta_dif = arctan2(dif[1], dif[0])
        theta_right = arctan2(right[1], right[0])
        theta_left = arctan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)

    if suitable_V:
        vA_post = min(suitable_V, key=lambda v: distance(v, vA))
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0] + pA[0] - p_0[0], new_v[1] + pA[1] - p_0[1]]
            theta_dif = arctan2(dif[1], dif[0])
            theta_right = arctan2(right[1], right[0])
            theta_left = arctan2(left[1], left[0])
    else:
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0] + pA[0] - p_0[0], unsuit_v[1] + pA[1] - p_0[1]]
                theta_dif = arctan2(dif[1], dif[0])
                theta_right = arctan2(right[1], right[0])
                theta_left = arctan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif - 0.5 * (theta_left + theta_right))
                    if abs(dist * sin(small_theta)) >= rad:
                        rad = abs(dist * sin(small_theta))
                    big_theta = arcsin(abs(dist * sin(small_theta)) / rad)
                    dist_tg = abs(dist * cos(small_theta)) - abs(rad * cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0
                    tc_v = dist_tg / distance(dif, [0, 0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc) + 0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key=lambda v: ((WT / tc_V[tuple(v)]) + distance(v, vA)))
    return vA_post


def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left < 0) and (theta_right > 0):
            theta_left += 2 * PI
            if theta_dif < 0:
                theta_dif += 2 * PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left > 0) and (theta_right < 0):
            theta_right += 2 * PI
            if theta_dif < 0:
                theta_dif += 2 * PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False


def check_distance(p1, p2, boundary=0.5):
    """To judge whether two points are under boundary"""
    if distance(p1, p2) < boundary:
        return True
    else:
        return False


def calcu_desire_V(robot, work_station, max_v, chose):
    desire_V = []
    for i in range(len(robot)):
        differ_x = [work_station[i][j] - robot[i][j] for j in range(2)]
        norm = distance(differ_x, [0, 0])
        norm_differ_x = [differ_x[j] * max_v/ norm for j in range(2)]
        #theta = arctan2(norm_differ_x[0],norm_differ_x[1])
        #norm_differ_x = [norm_differ_x[0]*cos(theta),norm_differ_x[1]*sin(theta)]
        desire_V.append(norm_differ_x[:])
        "When robot get to the workstation,velocity should be set zero"
        if check_distance(robot[i], work_station[i], 0.1):
            desire_V[i][0] = 0
            desire_V[i][1] = 0
            chose = 1
    return desire_V,chose
