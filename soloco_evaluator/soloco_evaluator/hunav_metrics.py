from importlib.resources import path
from tokenize import group
import numpy as np
import math
import sys
import rclpy
from geometry_msgs.msg import Pose
from soloco_interfaces.msg import TrackedPerson
from soloco_evaluator.sfm import SFM
from tf_transformations import euler_from_quaternion

# Teaching Robot Navigation Behaviors to Optimal RRT Planners
# Noé Pérez-Higueras, Fernando Caballero & Luis Merino

def euclidean_distance(pose, pose1):
    return math.sqrt((pose.position.x - pose1.position.x)**2 + (pose.position.y - pose1.position.y)**2)

def indicator_function(norm, k):
    if k == 'intimate':
        if norm < 0.3:
            return 1
        else:
            return 0
    elif k == 'personal':
        if norm >= 0.3 and norm < 0.5:
            return 1
        else:
            return 0
    elif k == 'social':
        if norm >= 0.5 and norm < 1.5:
            return 1
        else:
            return 0
    else:
        return 0
    
def get_time_stamps(agents, robot):
    time_list = []
    t0 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    for a in agents: 
        t = rclpy.time.Time.from_msg(a.header.stamp)
        dur = (t - t0).to_msg()
        s = float(dur.sec + dur.nanosec/1e9)
        time_list.append(s)
    return time_list

def total_time(agents, robot):
    t2 = rclpy.time.Time.from_msg(agents[len(agents)-1].header.stamp)
    t1 = rclpy.time.Time.from_msg(agents[0].header.stamp)
    dur = (t2 - t1).to_msg()
    secs = float(dur.sec + dur.nanosec/1e9)
    print('\nTime_to_reach_goal computed: %.2f secs' % secs)
    return [secs]

def robot_path_length(agents, robot):
    path_length = 0.0
    for i in range(len(robot)-1):
        path_length += euclidean_distance(robot[i+1].position, robot[i].position)
    print('Path_length computed: %.2f m' % path_length)
    return [path_length]

def cumulative_heading_changes(agents, robot):
    chc_list = [0.0]
    chc = 0
    for i in range(len(robot) - 1):
        norm = normalize_angle(robot[i].yaw - robot[i+1].yaw)
        if norm < 0.0:
            norm *= -1
        chc += norm
        chc_list.append(norm)

    print('Cumulative_heading_changes: %.2f rads' % chc)
    return [chc, chc_list]

def normalize_angle(ang):
    while (ang <= -math.pi): 
      ang += 2 * math.pi
    while (ang > math.pi):
      ang -= 2 * math.pi
    return ang

def avg_closest_person(agents, robot):
    min_dist_list = []
    avg_dist = 0
    for i in range(len(robot)):
        min_dist = 10000 
        for agent in agents[i].tracks:
            d = euclidean_distance(robot[i].position, agent.current_pose.pose) - robot[i].radius - agent.radius
            if(d < min_dist):
                min_dist = d
                if min_dist < 0.0:
                    min_dist = 0.0
        if(len(agents[i].tracks)>0):
            avg_dist += min_dist
            min_dist_list.append(min_dist)

    avg_dist = avg_dist/len(robot)
    print('Average_closest_person: %.2f m' % avg_dist)
    return [avg_dist, min_dist_list]


def minimum_distance_to_people(agents, robot):
    min_distance = list()

    for i in range(len(robot)):
        for agent in agents[i].tracks:
            d = euclidean_distance(robot[i].position, agent.current_pose.pose) - robot[i].radius - agent.radius
            if d<0.0:
                d = 0.0
            min_distance.append(d) 
    
    min_dist = min(min_distance)
    
    print('Minimum_distance_to_people: %.2f m' % min_dist)

    return [min_dist]

def space_intrusions(agents, robot, k):
    space_intrusions = 0
    space_intrusions_list = [0] * len(robot)

    for i in range(len(robot)):
        min_dist = 10000
        for agent in agents[i].tracks:
            d = euclidean_distance(robot[i].position, agent.current_pose.pose) - robot[i].radius - agent.radius
            if d < min_dist:
                min_dist = d
                if min_dist < 0.0:
                    min_dist = 0.0
        indicator = indicator_function(min_dist, k)
        if indicator == 1:
            space_intrusions += 1
            space_intrusions_list[i]=1

    space_intrusions = space_intrusions / len(robot)
    percentage = space_intrusions * 100.0

    return percentage, space_intrusions_list

def intimate_space_intrusions(agents, robot):
    percentage, slist =  space_intrusions(agents, robot, 'intimate')
    print('Intimate_space_intrusions: %.2f %% of the total time' % percentage)
    return [percentage, slist]

def personal_space_intrusions(agents, robot):
    percentage, slist =  space_intrusions(agents, robot, 'personal')
    print('Personal_space_intrusions: %.2f %% of the total time' % percentage)
    return [percentage, slist]
    
def social_space_intrusions(agents, robot):
    percentage, slist =  space_intrusions(agents, robot, 'social')
    print('Social_space_intrusions: %.2f %% of the total time' % percentage)
    return [percentage, slist]

# SEAN 2.0: Formalizing and Generating Social Situations for Robot Navigation
# Nathan Tsoi, Alec Xiang, Peter Yu, Samuel S. Sohn, Greg Schwartz, Subashri Ramesh, Mohamed Hussein, Anjali W. Gupta, Mubbasir Kapadia, and Marynel Vázquez

# The metrics Robot on Person Personal Distance Violation, Person on Robot Personal Distance Violation, Intimate Distance Violation and
# Person on Robot Intimate Distance Violation have already been implemented in the Personal_space_intrusions function.
# Instead of returning the number of times, it returns a percentage of distance violation.

def collisions(agents, robot):
    robot_coll_list = [0] * len(robot)
    person_coll_list = [0] * len(robot)
    robot_collisions = 0
    person_collisions = 0

    for i in range(len(robot)):
        for agent in agents[i].tracks:
            ori = agent.current_pose.pose.orientation
            quat = (ori.x, ori.y, ori.z, ori.w)
            agent_ori = euler_from_quaternion(quat)
            agent_yaw = agent_ori[2]
            if euclidean_distance(robot[i].position, agent.current_pose.pose) - robot[i].radius - agent.radius < 0.02:
                
                # Robot's angle
                nrx = (robot[i].position.position.x - agent.current_pose.pose.position.x) * math.cos(agent_yaw) + (robot[i].position.position.y - agent.current_pose.pose.position.y) * math.sin(agent_yaw)
                nry = -(robot[i].position.position.x - agent.current_pose.pose.position.x) * math.sin(agent_yaw) + (robot[i].position.position.y - agent.current_pose.pose.position.y) * math.cos(agent_yaw)
                alpha = math.atan2(nry, nrx)

                # Agent's angle
                nrx = (agent.current_pose.pose.position.x - robot[i].position.position.x) * math.cos(robot[i].yaw) + (agent.current_pose.pose.position.y - robot[i].position.position.y) * math.sin(robot[i].yaw)
                nry = -(agent.current_pose.pose.position.x - robot[i].position.position.x) * math.sin(robot[i].yaw) + (agent.current_pose.pose.position.y - robot[i].position.position.y) * math.cos(robot[i].yaw)
                alpha2 = math.atan2(nrx, nry)

                if abs(alpha) < abs(alpha2) and robot[i].linear_vel > agent.linear_vel:
                    robot_collisions += 1
                    robot_coll_list[i] = 1
                elif abs(alpha) > abs(alpha2) and robot[i].linear_vel < agent.linear_vel:
                    person_collisions += 1
                    person_coll_list[i] = 1
                elif abs(alpha) < abs(alpha2) and robot[i].linear_vel < agent.linear_vel:
                    #person_collision += 1
                    robot_collisions += 1
                    robot_coll_list[i] = 1
                elif abs(alpha) > abs(alpha2) and robot[i].linear_vel > agent.linear_vel:
                    #robot_collision += 1
                    person_collisions += 1
                    person_coll_list[i] = 1
                elif abs(alpha) == abs(alpha2) and robot[i].linear_vel == agent.linear_vel:
                    robot_collisions += 1
                    person_collisions += 1
                    robot_coll_list[i] = 1
                    person_coll_list[i] = 1

    return robot_collisions, person_collisions, robot_coll_list, person_coll_list

def robot_on_person_collision(agents, robot):
    collision = collisions(agents, robot)
    print('Robot_on_person_collision: %i ' % collision[0])

    return [collision[0], collision[2]]

def person_on_robot_collision(agents, robot):
    collision = collisions(agents, robot)
    print('Person_on_robot_collision: %i' % collision[1])

    return [collision[1], collision[3]]

def time_not_moving(agents, robot):
    not_moving = [0]*len(robot)
    time_step = total_time(agents, robot)[0]/len(agents)

    count = 0
    for index, r in enumerate(robot):
        if(r.linear_vel < 0.01 and abs(r.angular_vel < 0.02)):
            count=count+1
            not_moving[index]=1
    time_stopped = time_step*count
    print('Time stopped: %i secs' % time_stopped)
    return [time_stopped, not_moving]

def goal_reached(agents, robot):
    mind = 0.0
    if(len(robot[-1].goals)):
        for g in robot[-1].goals:
            d = euclidean_distance(robot[-1].position, g) - robot[-1].goal_radius
            if d<mind:
                return [True]
    return [False]

# SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
#ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University

def avg_robot_linear_speed(agents, robot):
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.linear_vel)
        speed += r.linear_vel

    speed = speed / len(robot)

    print('Average_robot_speed: %.2f m/s' % speed)
    return [speed, speed_list]

def avg_robot_angular_speed(agents, robot):
    speed_list = []
    speed = 0
    for r in robot:
        speed_list.append(r.angular_vel)
        speed += np.abs(r.angular_vel)

    speed = speed / len(robot)

    print('Average_robot_speed: %.2f rad/s' % speed)
    return [speed, speed_list]

def avg_acceleration(agents, robot):
    acceleration = 0
    acceleration_list = [0.0]
    for i in range(len(robot) - 1):
        dv = robot[i+1].linear_vel - robot[i].linear_vel
        tf = rclpy.time.Time.from_msg(agents[i+1].header.stamp)
        ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
        dur = (tf - ti).to_msg()
        dt = float(dur.sec + dur.nanosec/1e9)
        if dt != 0.0:
            accel = dv/dt
            acceleration += np.abs(accel)
            acceleration_list.append(accel)
        else:
            acceleration_list.append(0.0)

    acceleration = acceleration / len(robot)

    print('Average_robot_acceleration: %.5f m/s^2' % acceleration)

    return [acceleration, acceleration_list]

def avg_overacceleration(agents, robot):
    jerk = 0
    jerk_list = [0.0]
    for i in range(len(robot) - 1):
        dv = robot[i+1].linear_vel - robot[i].linear_vel
        tf = rclpy.time.Time.from_msg(agents[i+1].header.stamp)
        ti = rclpy.time.Time.from_msg(agents[i].header.stamp)
        dur = (tf - ti).to_msg()
        dt = float(dur.sec + dur.nanosec/1e9)
        if dt != 0.0:
            acceleration = dv/dt
            jerk += np.abs(acceleration/dt)
            jerk_list.append(acceleration/dt)
        else:
            jerk_list.append(0.0)

    jerk = jerk / len(robot)

    print('Average_robot_jerk(over_acceleration): %.5f m/s^3' % jerk)

    return [jerk, jerk_list]

# Learning a Group-Aware Policy for Robot Navigation
# Kapil Katyal ∗1,2 , Yuxiang Gao ∗2 , Jared Markowitz 1 , Sara Pohland 3 , Corban Rivera 1 , I-Jeng Wang 1 , Chien-Ming Huang 2

# metrics based on social force model

# cumulative modulus of the social force provoked 
# by the robot in the agents 
def social_force_on_agents(agents, robot):
    sfm = SFM()
    sf = 0.0
    sf_list = []
    for agts, rb in zip(agents, robot):
        f = sfm.modulusSocialForce2(rb, agts)
        sf += f
        sf_list.append(f)
    return [sf, sf_list]

# cumulative modulus of the social force provoked 
# by the agents in the robot
def social_force_on_robot(agents, robot):
    sfm = SFM()
    sf = 0.0
    sf_list = []
    for agts, rb in zip(agents, robot):
        f = sfm.modulusSocialForce(rb, agts)
        sf += f
        sf_list.append(f)
    return [sf, sf_list]

#TODO
def path_irregularity(agents, robot):
    pass

#TODO
def path_efficiency(agents, robot):
    pass

metrics = {
    'completed': goal_reached,
    # N. Perez-Higueras, F. Caballero, and L. Merino, “Teaching Robot Nav-
    # igation Behaviors to Optimal RRT Planners,” International Journal of
    # Social Robotics, vol. 10, no. 2, pp. 235–249, 2018.
    'time_to_reach_goal': total_time,
    'path_length': robot_path_length,
    'cumulative_heading_changes': cumulative_heading_changes,
    'avg_distance_to_closest_person': avg_closest_person,
    'minimum_distance_to_people': minimum_distance_to_people,
    'intimate_space_intrusions': intimate_space_intrusions,
    'personal_space_intrusions': personal_space_intrusions,
    'social_space_intrusions': social_space_intrusions,
    # N. Tsoi, A. Xiang, P. Yu, S. S. Sohn, G. Schwartz, S. Ramesh,
    # M. Hussein, A. W. Gupta, M. Kapadia, and M. V ́azquez, “Sean 2.0:
    # Formalizing and generating social situations for robot navigation,”
    # IEEE Robotics and Automation Letters, vol. 7, no. 4, pp. 11 047–
    # 11 054, 2022
    #   - 'Total Path Length' (meters): similar to 'path_length'
    #   - 'Path Irregularity': (radians): total rotations in the robot's 
    #       traveled path greater than the total rotations in the search-based 
    #       path from the starting pose.
    #   - 'Path Efficiency': (meters): ratio between robot's traveled path and 
    #       geodesic distance of the search-based path from the starting pose.

    # true when the robot's final pose is within a specified distance of the goal. 
    # The final distance threshold is easily adjustable by the user, but defaults 
    # to 1.2m.
    #   - 'Robot on Person Personal Distance Violation': number of times a robot 
    # approaches a person within the personal distance of the robot.
    # Similar to 'personal_space_intrusions'
    #   - 'Person on Robot Personal Distance Violation': number of times a person 
    # approaches the robot within the personal distance of the robot.
    #   - 'Intimate Distance Violation': number of times the robot approached within 
    # the intimate distance of a person.
    #   - 'Person on Robot Intimate Distance Violation': number of times a person 
    # approaches the robot within the intimate distance of the robot.
    'robot_on_person_collision': robot_on_person_collision,
    'person_on_robot_collision': person_on_robot_collision,
    'time_not_moving': time_not_moving,

    # SocNavBench: A Grounded Simulation Testing Framework for Evaluating Social Navigation
    #ABHIJAT BISWAS, ALLAN WANG, GUSTAVO SILVERA, AARON STEINFELD, and HENNY AD-MONI, Carnegie Mellon University
    'avg_robot_linear_speed': avg_robot_linear_speed,
    'avg_robot_angular_speed': avg_robot_angular_speed,
    'avg_acceleration': avg_acceleration,
    'avg_overacceleration': avg_overacceleration,

    # metrics based on Social Force Model employed in different papers
    'social_force_on_agents': social_force_on_agents,
    'social_force_on_robot': social_force_on_robot,
}




