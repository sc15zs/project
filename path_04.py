#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
Path planning in non-static environment: Kinodynamic RRT method algorithm
to push a movable obstacle out of the robot's way
'''
import math, time
import numpy as np
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import random
import Box2D
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, kinematicBody)


class World():
    ##---Function to calculate the next movements in all 8 possible directions---##
    def calculate_coordinates(x, y, target_x, target_y):
        coordinates = {}
        coordinates[np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y+1) - abs(target_y)))]  = (x, y + 1)  ## Move N
        coordinates[np.sqrt(np.square(abs(x-1) - abs(target_x)) + np.square(abs(y+1) - abs(target_y)))] = (x - 1, y + 1)  ## Move NW
        coordinates[np.sqrt(np.square(abs(x-1) - abs(target_x)) + np.square(abs(y) - abs(target_y)))]  = (x - 1, y)      ## Move W
        coordinates[np.sqrt(np.square(abs(x-1) - abs(target_x)) + np.square(abs(y-1) - abs(target_y)))] = (x - 1, y - 1)  ## Move SW
        coordinates[np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y-1) - abs(target_y)))]  = (x,     y - 1)  ## Move S
        coordinates[np.sqrt(np.square(abs(x+1) - abs(target_x)) + np.square(abs(y-1) - abs(target_y)))] = (x + 1, y - 1)  ## Move SE
        coordinates[np.sqrt(np.square(abs(x+1) - abs(target_x)) + np.square(abs(y) - abs(target_y)))]  = (x + 1, y)      ## Move E
        coordinates[np.sqrt(np.square(abs(x+1) - abs(target_x)) + np.square(abs(y+1) - abs(target_y)))] = (x + 1, y + 1)  ## Move NE
        return coordinates

    ##---Function to find the closest node in the graph to a target position---##
    def find_closest_node(graph, target_x, target_y):
        result_graph = {}
        counter = 0
        for key, value in sorted(graph.items()):
            x = value[0]
            y = value[1]
            distance = np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y) - abs(target_y)))
            result_graph[counter] = distance
            counter+=1
            ##---Find node in graph with minimum distance value and return its key---##
        min_node_number = min(result_graph, key=result_graph.get)
        return min_node_number


    ##---constants---##
    PPM = 20.0  # pixels per meter
    TARGET_FPS = 60
    TIME_STEP = 1.0 / TARGET_FPS
    vel_iters, pos_iters = 6, 2
    SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 800

    ##---pygame setup---##
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    pygame.display.set_caption('Path finding: Kinodynamic RRT 1.')
    clock = pygame.time.Clock()

    ##---Create the world---##
    my_world = world(gravity=(0, 0), doSleep=True)

    ##---Vertices for the robot body---##
    vertices2 = [(-0.5, -0.2), (-0.2, -0.5), (0.2, -0.5), (0.5, -0.2), (0.5, 0.2), (0.2, 0.5), (-0.2, 0.5), (-0.5, 0.2)]
    ##---Vertices for the moveable object---##
    vertices3 = [(-1.0, -0.4), (-0.4, -1.0), (0.4, -1.0), (1.0, -0.4), (1.0, 0.4), (0.4, 1.0), (-0.4, 1.0), (-1.0, 0.4)]

    # ##---Static obstacles of the environment---##
    # wall_bottom = my_world.CreateStaticBody(position=(30, 0.1), shapes=polygonShape(box=(30, 0.1)), )
    # wall_top = my_world.CreateStaticBody(position=(30, 40), shapes=polygonShape(box=(30, 0.1)), )
    # wall_left = my_world.CreateStaticBody(position=(0, 20), shapes=polygonShape(box=(20, 0.1)), angle=math.pi/2)
    # wall_right = my_world.CreateStaticBody(position=(60, 20), shapes=polygonShape(box=(20, 0.1)), angle=math.pi/2)
    # wall_bottom.userData = {'color': 'obstacle'}
    # wall_top.userData = {'color': 'obstacle'}
    # wall_left.userData = {'color': 'obstacle'}
    # wall_right.userData = {'color': 'obstacle'}

    #---AREA 1---##
    obstacle_1 = my_world.CreateStaticBody(position=(5, 12.5), shapes=polygonShape(box=(5, 2.5)))
    obstacle_2 = my_world.CreateStaticBody(position=(39.0, 25), shapes=polygonShape(box=(7, 1.5)), angle=math.pi / 2)
    obstacle_3 = my_world.CreateStaticBody(position=(33, 25), shapes=polygonShape(box=(7, 1.5)), angle=-math.pi / 2)
    obstacle_4 = my_world.CreateStaticBody(position=(36, 31), shapes=polygonShape(box=(4.5, 1.5)), angle=0)
    obstacle_1.userData = {'color': 'obstacle'}
    obstacle_2.userData = {'color': 'obstacle'}
    obstacle_3.userData = {'color': 'obstacle'}
    obstacle_4.userData = {'color': 'obstacle'}

    # ##---AREA 2---##
    # obstacle_1 = my_world.CreateStaticBody(position=(5, 12.5), shapes=polygonShape(box=(5, 2.5)))
    # obstacle_2 = my_world.CreateStaticBody(position=(39.0, 25), shapes=polygonShape(box=(7, 1.5)), angle=math.pi / 2)
    # obstacle_3 = my_world.CreateStaticBody(position=(33.0, 22.5), shapes=polygonShape(box=(4.5, 1.5)), angle=-math.pi / 2)
    # obstacle_4 = my_world.CreateStaticBody(position=(32, 31), shapes=polygonShape(box=(8.5, 1.5)), angle=0)
    # obstacle_5 = my_world.CreateStaticBody(position=(27.5, 26), shapes=polygonShape(box=(4.0, 1.0)), angle=0)
    # obstacle_1.userData = {'color': 'obstacle'}
    # obstacle_2.userData = {'color': 'obstacle'}
    # obstacle_3.userData = {'color': 'obstacle'}
    # obstacle_4.userData = {'color': 'obstacle'}
    # obstacle_5.userData = {'color': 'obstacle'}


    ##---Dictionary to hold node numbers as keys and neighbouring node numbers and their distances as values---##
    my_graph = {}
    ##---Dictionary to hold node numbers as keys and x,y coordinates as values---##
    my_grid = {}
    ##---Settings---##
    robot_start_position = (36, 3)
    robot_rotation = 0
    goal_position = (36.0, 28.0)
    moveable_object_position = (36, 17)
    moveable_object_rotation = 0

    my_grid[0] = [robot_start_position[0], robot_start_position[1], moveable_object_position[0], moveable_object_position[1]]
    my_graph[0] = {0: 0}
    node_counter = 1
    start_time = time.clock()

    moveable_object_new_position = moveable_object_position
    ##---Until we don't find the goal---##
    while (True):
        ##---Set maximum number of nodes allowed in the tree---##
        if (node_counter < 1500):
            if (node_counter % 8 == 0):
                ##---FOR ROBOT: The goal position is the "random" point---##
                robot_x = goal_position[0]
                robot_y = goal_position[1]
            else:
                ##---FOR ROBOT: Generate random x and y coordinates and round them to two decimal places---##
                robot_x = round(random.uniform(0.0, 60.0), 2)
                robot_y = round(random.uniform(0.0, 40.0), 2)
            robot_random_position = (robot_x, robot_y)
            ##---FOR MOVEABLE OBSTACLE: Generate random x and y coordinates and round them to two decimal places---##
            obstacle_x = round(random.uniform(0.0, 60.0), 2)
            obstacle_y = round(random.uniform(0.0, 40.0), 2)
            obstacle_random_position = (obstacle_x, obstacle_y)
            ##---FOR ROBOT: Find closest node in graph to the random point---##
            closest_node_number = find_closest_node(my_grid, robot_x, robot_y)

            ##---FOR ROBOT: Get the 8 possible movements from this node---##
            coordinates = calculate_coordinates(my_grid[closest_node_number][0], my_grid[closest_node_number][1], robot_x, robot_y)
            total_distances = {}

            ##--- Check the possible moves for collision one by one: starting with the point closest to the random point---##
            for n in sorted(coordinates):
                robot_next_x_coordinate = coordinates[n][0]
                robot_next_y_coordinate = coordinates[n][1]
                robot_new_position = (robot_next_x_coordinate, robot_next_y_coordinate)

                ##---CREATE ROBOT---##
                current_robot_position_x = my_grid[closest_node_number][0]
                current_robot_position_y = my_grid[closest_node_number][1]
                body = my_world.CreateDynamicBody(position=(current_robot_position_x, current_robot_position_y))
                box = body.CreatePolygonFixture(vertices=vertices2, density=1)
                ##---CREATE MOVEABLE OBJECT---##
                current_moveable_object_position_x = my_grid[closest_node_number][2]
                current_moveable_object_position_y = my_grid[closest_node_number][3]
                body2 = my_world.CreateDynamicBody(position=(current_moveable_object_position_x, current_moveable_object_position_y))
                box = body2.CreatePolygonFixture(vertices=vertices3, density=0.3)

                ##---Move the robot to its next coordinate---##
                loop_counter = 0
                while(loop_counter<150 and  ( ( abs(body.worldCenter.x - (robot_next_x_coordinate))>0.03 ) or ( abs(body.worldCenter.y-(robot_next_y_coordinate))>0.03 ) )  ):
                    body.linearVelocity = (robot_next_x_coordinate-current_robot_position_x, robot_next_y_coordinate-current_robot_position_y)
                    my_world.Step(TIME_STEP, 6, 2)
                    loop_counter+=1
                body.linearVelocity = (0, 0)
                body2.linearVelocity = (0, 0)

                ##---If the robot managed to reach its destination---##
                if (loop_counter<140):
                    moveable_object_new_position = (body2.worldCenter.x, body2.worldCenter.y)
                    moveable_object_rotation = body2.angle
                    ##---Calculate distance of robot position from the random target robot position---##
                    distance_x = robot_new_position[0] - robot_random_position[0]
                    distance_y = robot_new_position[1] - robot_random_position[1]
                    robot_distance = np.sqrt(np.square(distance_x) + np.square(distance_y))
                    ##---Calculate distance of moveable object from the random target object position---##
                    moveable_object_distance_x = moveable_object_new_position[0] - obstacle_random_position[0]
                    moveable_object_distance_y = moveable_object_new_position[1] - obstacle_random_position[1]
                    moveable_object_distance = np.sqrt(np.square(moveable_object_distance_x) + np.square(moveable_object_distance_y))
                    ##---Sum up the robot distance and the moveable object distance---##
                    total_distance = round(robot_distance + moveable_object_distance, 6)
                    ##---Store total_distance and corresponding robot coordinates and moveable object coordinates---##
                    total_distances[total_distance] = [robot_new_position[0], robot_new_position[1], moveable_object_new_position[0], moveable_object_new_position[1], robot_rotation, moveable_object_rotation]
                    my_world.DestroyBody(body)
                    my_world.DestroyBody(body2)
                ##---The robot didn't reach its destination---##
                else:
                    my_world.DestroyBody(body)
                    my_world.DestroyBody(body2)
            sorted_distance = {}
            ##---Find the shortest distance in the total_distances dictionary---##
            if (len(total_distances) != 0):
                ##---Find the lowest distance in the total_distances dictionary keys---##
                first_key = sorted(total_distances)[0]
                ##---Find the x and y coordinates for both the robot and the moveable object---##
                ##---(this state has the least distance between the robot, object and their corresponding random goals---##
                closest_point = total_distances[first_key]
                robot_rotation_to_store = closest_point[4]
                moveable_object_rotation_to_store = closest_point[5]

                ##---Store in the graph the previous (parent) node---##
                my_graph[node_counter] = closest_node_number
                ##---Store new node number,  x and y coordinates of robot,  x and y coordinates of moveable obstacle, rotation of the robot and obstacle---##
                my_grid[node_counter] = [closest_point[0], closest_point[1], closest_point[2], closest_point[3], robot_rotation_to_store, moveable_object_rotation_to_store]

                node_counter += 1
                if (node_counter % 100 == 0):
                    print(node_counter)

            ##---If we are close enough to the goal---##
            if ((np.square(goal_position[0] - closest_point[0]) < 0.4) and (np.square(goal_position[1] - closest_point[1]) < 0.4)):
                found_goal_position = my_graph[node_counter-1]
                goal_node_number = node_counter-1
                break
        ##---If maximum number of nodes has been reached---##
        else:
            ##---Delete my_grid and my_graph and start building new ones---##
            my_grid = {}
            my_grid[0] = [robot_start_position[0], robot_start_position[1], moveable_object_position[0], moveable_object_position[1], robot_rotation, moveable_object_rotation]
            my_graph = {}
            my_graph[0] = 0
            node_counter = 1

    ##---Trace back parent nodes of each node to get the path between the start and the goal---##
    node_number = goal_node_number
    shortest_path = []
    print("node_number: ", node_number)
    while (True):
        shortest_path.append(node_number)
        previous_node_number = my_graph.get(node_number)
        node_number = previous_node_number
        if (node_number == 0):
            shortest_path.append(node_number)
            break

    shortest_path.reverse()
    print("shortest path: ")
    print(shortest_path)

    ##---Create static bodies to mark the nodes in the tree with small "dots"---##
    for key, value in my_grid.items():
        x = value[0]
        y = value[1]
        body3 = my_world.CreateStaticBody(position=(x,y))
        box3 = body3.CreatePolygonFixture(box=(0.098, 0.098), density=1)
        box3.sensor = True
        body3.userData = {'color': 'node_marker'}

    ##---Create kinematic bodies to mark the shortest path with small "dots"---##
    for number in shortest_path:
        x = my_grid[number][0]
        y = my_grid[number][1]
        body = my_world.CreateKinematicBody(position=(x, y))
        box = body.CreatePolygonFixture(box=(0.098, 0.098), density=1, friction=0.3)
        box.sensor = True
        body.userData = {'color': 'shortest_path'}

    ##---Create static body to mark the goal position---##
    body2 = my_world.CreateStaticBody(position=goal_position)
    box2 = body2.CreatePolygonFixture(vertices=vertices2, density=1)
    box2.sensor = True
    body2.userData = {'color': 'goal_position'}

    ##---Create static body to mark the start position---##
    body3 = my_world.CreateStaticBody(position=robot_start_position)
    box3 = body3.CreatePolygonFixture(vertices=vertices2, density=1)
    box3.sensor = True
    body3.userData = {'color': 'start_position'}

    ##---Create a new moveable object at the moveable obstacle position---##
    body = my_world.CreateDynamicBody(position=moveable_object_position)
    box = body.CreatePolygonFixture(vertices=vertices3, density=0.3)
    body.userData = {'color': 'moveable_obstacle'}

    ##---Create a dynamic body for the robot at the start position---##
    dynamic_body = my_world.CreateDynamicBody(position=robot_start_position, angle=0)
    box = dynamic_body.CreatePolygonFixture(vertices=vertices2, density=1, friction=0.3)
    dynamic_body.userData = {'color': 'robot'}

    ##---Define colours for robot, obstacles, start- and goal positions, node position markers---##
    colors = {
        'obstacle': (115, 115, 115),
        'robot': (165, 0, 0),
        'moveable_obstacle': (77, 166, 255),
        'start_position': (255, 140, 26),
        'goal_position': (204, 255, 153),
        'node_marker': (0, 77, 0),
        'shortest_path': (255, 128, 223),
    }

    ##---Calculate and print runtime---##
    end_time = time.clock()
    time_needed = end_time - start_time
    print("Time needed: %f s." % time_needed)





##---main game loop---##
my_world = World()
grid = my_world.my_grid

running = True
goal_reached = False
counter = 0
target_coordinate_list = []

##---Get path---##
targetlist = my_world.shortest_path

##---Create list of the shortest path cells: for each cell get the x coordinate, y coordinate and cell number---##
for n in range(len(targetlist)):
    list = grid[targetlist[n]]
    list.append(targetlist[n])
    target_coordinate_list.append(list)
# print("targets:::::::::::")
# print(target_coordinate_list)
# print("Length of targetlist: ", len(targetlist))
goal_node = len(targetlist)-1

while running:
    ##---Check the event queue of the game---##
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            running = False
    my_world.screen.fill((0, 0, 0, 0))

    robot = my_world.my_world.bodies[-1]
    moveable_obstacle = my_world.my_world.bodies[-2]
    ##---Draw the world---##
    for body in (my_world.my_world.bodies):
        for fixture in body.fixtures:
            shape = fixture.shape
            vertices = [(body.transform * v) * my_world.PPM for v in shape.vertices]
            vertices = [(v[0], my_world.SCREEN_HEIGHT - v[1]) for v in vertices]
            pygame.draw.polygon(my_world.screen, my_world.colors[body.userData['color']], vertices)
    my_world.my_world.Step(my_world.TIME_STEP, 10, 10)
    ##---Flip the screen and try to keep at the target FPS---##
    pygame.display.flip()
    my_world.clock.tick(my_world.TARGET_FPS)

    ##---The target is the actual element of the target coordinate list---##
    target = target_coordinate_list[counter]

    ##--Until we haven't reached the goal position----##
    if (goal_reached == False):
        ##---If the difference between the x and y coordinates is larger than a certain threshold---##
        if (np.square(target[0] - robot.worldCenter.x) > 0.01 or np.square(target[1] - robot.worldCenter.y) > 0.01):
            ##---Calculate the difference in x and y directions between the actual and the desired position---##
            direction_x = target[0] - robot.worldCenter.x
            direction_y = target[1] - robot.worldCenter.y
            ##---Calculate the distance---##
            distance = np.sqrt(np.square(direction_x)+np.square(direction_y))
            ##---We don't want a fluctuating robot speed: it would go quickly when the distance is large from---##
            ##---the next target and gets slower and slower as the distance decreases, so we set the speed component---##
            ##---of the linearVelocity of the robot inversely proportional to the distance to have a nice, smooth execution---##
            speed = 1.9 * 1/distance

            robot.linearVelocity = (speed*direction_x, speed*direction_y)
            my_world.my_world.Step(my_world.TIME_STEP, 10, 10)
        ##---If robot reaches the actual target point---##
        else:
            counter+=1
        ##---If robot reaches the final goal node (within a certain threshold)---##
        if (np.square(target_coordinate_list[goal_node][0] - robot.worldCenter.x) < 0.01 and np.square(target_coordinate_list[goal_node][1] - robot.worldCenter.y) < 0.01):
            goal_reached=True
        ##---Stop the robot and the obstacle (if it was in drift for any reason)---##
        robot.linearVelocity = (0, 0)
        robot.angularVelocity = (0)
        moveable_obstacle.linearVelocity = (0, 0)
        moveable_obstacle.angularVelocity = (0)


# ##---If want to show the starting situation use this part and comment out previous while loop---##
# while running:
#      ###Check the event queue
#     for event in pygame.event.get():
#         if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
#             # The user closed the window or pressed escape
#             running = False
#
#     my_world.screen.fill((0, 0, 0, 0))
#     robot = my_world.my_world.bodies[-1]
#     moveable_obstacle = my_world.my_world.bodies[-2]
#
#     # Draw the world
#     for body in (my_world.my_world.bodies):  # or: world.bodies
#         # The body gives us the position and angle of its shapes
#         for fixture in body.fixtures:
#             shape = fixture.shape
#             vertices = [(body.transform * v) * my_world.PPM for v in shape.vertices]
#             vertices = [(v[0], my_world.SCREEN_HEIGHT - v[1]) for v in vertices]
#
#             pygame.draw.polygon(my_world.screen, my_world.colors[body.userData['color']], vertices)
#     my_world.my_world.Step(my_world.TIME_STEP, 10, 10)
#     # print(len(my_world.my_world.bodies))
#
#     pygame.display.flip()
#     my_world.clock.tick(my_world.TARGET_FPS)
#


pygame.quit()
print('Done!')
