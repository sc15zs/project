#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math, time, random
from datetime import datetime
import numpy as np
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, kinematicBody)
import Box2D
from Box2D import b2FixtureDef, b2CircleShape


class World(object):
    def calculate_coordinates(self, x, y, target_x, target_y):
        coordinates = {}
        ##---For each movement the dictionary key is the distance---##
        ##---For each movement the dictionary will contain the new x and y coordinates and the angle to turn---##
        coordinates[np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y+1) - abs(target_y)))]  = (x, y + 1, 0)  ## Move N
        coordinates[np.sqrt(np.square(abs(x-1) - abs(target_x)) + np.square(abs(y+1) - abs(target_y)))] = (x - 1, y + 1, math.pi/4)  ## Move NW
        coordinates[np.sqrt(np.square(abs(x-1) - abs(target_x)) + np.square(abs(y) - abs(target_y)))]  = (x - 1, y, math.pi/2)      ## Move W
        coordinates[np.sqrt(np.square(abs(x-1) - abs(target_x)) + np.square(abs(y-1) - abs(target_y)))] = (x - 1, y - 1, 3*math.pi/4)  ## Move SW
        coordinates[np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y-1) - abs(target_y)))]  = (x, y - 1, math.pi/2)  ## Move S
        coordinates[np.sqrt(np.square(abs(x+1) - abs(target_x)) + np.square(abs(y-1) - abs(target_y)))] = (x + 1, y - 1, -3*math.pi/4)  ## Move SE
        coordinates[np.sqrt(np.square(abs(x+1) - abs(target_x)) + np.square(abs(y) - abs(target_y)))]  = (x + 1, y, -math.pi/2)      ## Move E
        coordinates[np.sqrt(np.square(abs(x+1) - abs(target_x)) + np.square(abs(y+1) - abs(target_y)))] = (x + 1, y + 1, -math.pi/4)  ## Move NE

        # coordinates[np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y+2) - abs(target_y)))]  = (x, y + 2, 0)  ## Move N
        # coordinates[np.sqrt(np.square(abs(x-2) - abs(target_x)) + np.square(abs(y+2) - abs(target_y)))] = (x - 2, y + 2, math.pi/4)  ## Move NW
        # coordinates[np.sqrt(np.square(abs(x-2) - abs(target_x)) + np.square(abs(y) - abs(target_y)))]  = (x - 2, y, math.pi/2)      ## Move W
        # coordinates[np.sqrt(np.square(abs(x-2) - abs(target_x)) + np.square(abs(y-2) - abs(target_y)))] = (x - 2, y - 2, 3*math.pi/4)  ## Move SW
        # coordinates[np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y-2) - abs(target_y)))]  = (x, y - 2, math.pi/2)  ## Move S
        # coordinates[np.sqrt(np.square(abs(x+2) - abs(target_x)) + np.square(abs(y-2) - abs(target_y)))] = (x + 2, y - 2, -3*math.pi/4)  ## Move SE
        # coordinates[np.sqrt(np.square(abs(x+2) - abs(target_x)) + np.square(abs(y) - abs(target_y)))]  = (x + 2, y, -math.pi/2)      ## Move E
        # coordinates[np.sqrt(np.square(abs(x+2) - abs(target_x)) + np.square(abs(y+2) - abs(target_y)))] = (x + 2, y + 2, -math.pi/4)  ## Move NE
        return coordinates

    def calculate_distance (x, y, target_x, target_y):
        return np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y) - abs(target_y)))

    def find_closest_node(self, graph, robot_target_x, robot_target_y, obstacle_target_x, obstacle_target_y):
        result_graph = {}
        counter = 0
        for key, value in sorted(graph.items()):
            robot_x = value[0]
            robot_y = value[1]
            obstacle_x = value[2]
            obstacle_y = value[3]
            robot_distance = np.sqrt(np.square(abs(robot_x) - abs(robot_target_x)) + np.square(abs(robot_y) - abs(robot_target_y)))
            obstace_distance = np.sqrt(np.square(abs(obstacle_x) - abs(obstacle_target_x)) + np.square(abs(obstacle_y) - abs(obstacle_target_y)))
            distance = 0.4*robot_distance + 0.6*obstace_distance
            result_graph[counter] = distance
            counter+=1
            ##---Find node in graph with minimum distance value and return its key---##
        min_node_number = min(result_graph, key=result_graph.get)
        return min_node_number


    def __init__(self, rob_x, rob_y, rob_rot, obs_x, obs_y, obs_rot, num_run, tim_lis):
        ##---constants---##
        self.PPM = 20.0  # pixels per meter
        self.TARGET_FPS = 60
        self.TIME_STEP = 1.0 / self.TARGET_FPS
        self.SCREEN_WIDTH, self.SCREEN_HEIGHT = 1200, 800
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT), 0, 32)
        self.gravity = (0, 0)
        self.doSleep = True
        # ##---pygame setup---##
        pygame.display.set_caption('Path finding')
        self.clock = pygame.time.Clock()

        self.robotCategory = 0x0001
        self.objectCategory = 0x0002
        self.robotMask = 0xFFFF ^ self.robotCategory
        self.objectMask = 0xFFFF ^ self.objectCategory

        # Create the world
        self.my_world = world(gravity=(0, 0), doSleep=self.doSleep)
        self.number_of_runs = num_run
        ##---Create a new list for storing the times of executions if it is the first run---##
        #if (self.number_of_runs == 0): time_list = []
        self.time_list = tim_lis

        ##---Vertices for the moveable obstacle---##
        vertices1 = [(-0.4, -0.2), (-0.2, -0.4), (0.2, -0.4), (0.4, -0.2), (0.4, 0.2), (0.2, 0.4), (-0.2, 0.4), (-0.4, 0.2)]

        ##---Vertices for the robot body---##
        vertices2 = [(-0.5, -0.25), (-0.25, -0.5), (0.25, -0.5), (0.5, -0.25), (0.5, 0.25), (0.25, 0.5), (-0.25, 0.5), (-0.5, 0.25)]
        horizontal_arm_base = [(-1.2, 0.5), (1.2, 0.5), (1.2, 0.3), (-1.2, 0.3)]
        left_long_arm = [(-1.3, 0.3), (-1.2, 0.3), (-1.1, 1.6), (-1.2, 1.6)]
        right_long_arm = [(1.3, 0.3), (1.2, 0.3), (1.1, 1.6), (1.2, 1.6)]
        left_short_arm = [(-0.8, 1.9), (-0.7, 1.9), (-1.1, 1.6), (-1.2, 1.6)]
        right_short_arm = [(0.8, 1.9), (0.7, 1.9), (1.1, 1.6), (1.2, 1.6)]
        test_robot = [(-2.0, 2.0), (-2.0, -2.0), (2.0, -2.0), (2.0, 2.0)]

        # ##---Create static bodies for walls and obstacles---##
        # wall_bottom = self.my_world.CreateStaticBody(position=(30, 0.1), shapes=polygonShape(box=(30, 0.1)), )
        # wall_top = self.my_world.CreateStaticBody(position=(30, 40), shapes=polygonShape(box=(30, 0.1)), )
        # wall_left = self.my_world.CreateStaticBody(position=(0, 20), shapes=polygonShape(box=(20, 0.1)), angle=math.pi/2)
        # wall_right = self.my_world.CreateStaticBody(position=(60, 20), shapes=polygonShape(box=(20, 0.1)), angle=math.pi/2)
        # wall_bottom.userData = {'color': 'obstacle'}
        # wall_top.userData = {'color': 'obstacle'}
        # wall_left.userData = {'color': 'obstacle'}
        # wall_right.userData = {'color': 'obstacle'}

        ##---AREA 1---##
        obstacle_1 = self.my_world.CreateStaticBody(position=(34, 12.0), shapes=polygonShape(box=(18, 0.5)))
        obstacle_2 = self.my_world.CreateStaticBody(position=(19.0, 33.5), shapes=polygonShape(box=(6, 0.5)), angle=math.pi / 2)
        obstacle_1.userData = {'color': 'obstacle'}
        obstacle_2.userData = {'color': 'obstacle'}

        ##---AREA 2---##
        # obstacle_1 = my_world.CreateStaticBody(position=(5, 12.5), shapes=polygonShape(box=(5, 2.5)))
        # obstacle_2 = my_world.CreateStaticBody(position=(39.0, 25), shapes=polygonShape(box=(7, 1.5)), angle=math.pi / 2)
        # obstacle_3 = my_world.CreateStaticBody(position=(33.0, 22.5), shapes=polygonShape(box=(4.5, 1.5)), angle=-math.pi / 2)
        # obstacle_4 = my_world.CreateStaticBody(position=(32, 31), shapes=polygonShape(box=(8.5, 1.5)), angle=0)
        # obstacle_5 = my_world.CreateStaticBody(position=(27.5, 26), shapes=polygonShape(box=(4.0, 1.0)), angle=0)


        ##---Dictionary to hold node numbers as keys their parent node number as values---##
        my_graph = {}
        ##---Dictionary to hold node numbers as keys and x,y coordinates as values---##
        self.my_grid = {}
        ##---Settings---##
        timeStep = 1.0 / 60
        # vel_iters, pos_iters = 6, 2
        goal_position = (6.0, 34.0)
        moveable_object_position = (obs_x, obs_y)
        moveable_object_rotation = obs_rot
        robot_start_position = (rob_x, rob_y)
        robot_rotation = rob_rot

        self.my_grid[0] = [robot_start_position[0], robot_start_position[1], moveable_object_position[0], moveable_object_position[1], robot_rotation, moveable_object_rotation]
        my_graph[0] = 0
        node_counter = 1
        start_time = time.clock()

        moveable_object_new_position = moveable_object_position
        while (True):
            if (node_counter < 2500):
                if (node_counter % 8 == 0):
                    ##---FOR MOVEABLE OBSTACLE: The goal position is the "random" point---##
                    x = goal_position[0]
                    y = goal_position[1]
                else:
                    ##---FOR MOVEABLE OBSTACLE: Generate random x and y coordinates and round them to two decimal places---##
                    x = round(random.uniform(0.0, 60.0), 2)
                    y = round(random.uniform(0.0, 40.0), 2)
                obstacle_random_position = (x, y)

                ##---FOR ROBOT: Generate random x and y coordinates and round them to two decimal places---##
                if (node_counter % 6 == 0):
                    robot_x = moveable_object_new_position[0]
                    robot_y = moveable_object_new_position[1]
                else:
                    robot_x = round(random.uniform(0.0, 60.0), 2)
                    robot_y = round(random.uniform(0.0, 40.0), 2)
                robot_random_position = (robot_x, robot_y)
                # ##---FOR ROBOT: Find closest node in graph to the random point---##
                closest_node_number = self.find_closest_node(self.my_grid, robot_x, robot_y, x, y)

                ##---FOR ROBOT: Get the 8 possible movements from this node---##
                coordinates = self.calculate_coordinates(self.my_grid[closest_node_number][0], self.my_grid[closest_node_number][1], robot_x, robot_y)

                # total_distances = {}
                # ##--- Check the possible moves for collision one by one: starting with the point closest to the random point---##
                my_counter=0
                ##---Check for collision only with the robot: with a fully closed body to avoid problems while turning with the real robot---##
                for n in sorted(coordinates):
                    ##---CREATE ROBOT---##
                    current_robot_position_x = coordinates[n][0]
                    current_robot_position_y = coordinates[n][1]
                    current_robot_position = (current_robot_position_x, current_robot_position_y)
                    body = self.my_world.CreateDynamicBody(position=(current_robot_position_x, current_robot_position_y), angle=0)
                    body.CreatePolygonFixture(vertices=test_robot, density=1, friction=1, isSensor=False)
                    for i in range(3):
                        # Instruct the world to perform a single step of simulation. It is
                        self.my_world.Step(timeStep, 10, 6)
                    if (current_robot_position != body.position):
                        del coordinates[n]
                        my_counter+=1

                    #print("n: ", n)
                    self.my_world.DestroyBody(body)

                # if node_counter<2:
                #     print("coordinates dict: ", coordinates)

                #print ("deleted positions: ", my_counter)
                total_distances = {}
                ##--- Check the possible moves for collision one by one: starting with the point closest to the random point---##
                for n in sorted(coordinates):
                    robot_next_x_coordinate = coordinates[n][0]
                    robot_next_y_coordinate = coordinates[n][1]
                    robot_rotation = coordinates[n][2]
                    robot_new_position = (robot_next_x_coordinate, robot_next_y_coordinate)

                    ##---CREATE ROBOT---##
                    current_robot_position_x = self.my_grid[closest_node_number][0]
                    current_robot_position_y = self.my_grid[closest_node_number][1]
                    body = self.my_world.CreateDynamicBody(position=(current_robot_position_x, current_robot_position_y), angle=robot_rotation)
                    body.CreatePolygonFixture(vertices=vertices2, density=1, friction=1, isSensor=True)
                    body.CreatePolygonFixture(vertices=horizontal_arm_base, density=1, friction=1, isSensor=False)
                    body.CreatePolygonFixture(vertices=left_long_arm, density=1, friction=1, isSensor=False)
                    body.CreatePolygonFixture(vertices=right_long_arm, density=1, friction=1, isSensor=False)
                    body.CreatePolygonFixture(vertices=left_short_arm, density=1, friction=1, isSensor=False)
                    body.CreatePolygonFixture(vertices=right_short_arm, density=1, friction=1, isSensor=False)
                    ##---CREATE MOVEABLE OBJECT---##
                    current_moveable_object_position_x = self.my_grid[closest_node_number][2]
                    current_moveable_object_position_y = self.my_grid[closest_node_number][3]
                    current_moveable_object_rotation = self.my_grid[closest_node_number][5]
                    body2 = self.my_world.CreateDynamicBody(position=(current_moveable_object_position_x, current_moveable_object_position_y), angle=current_moveable_object_rotation)
                    body2.CreatePolygonFixture(vertices=vertices1, density=0.3)

                    loop_counter = 0
                    while(loop_counter<150 and  ( ( abs(body.worldCenter.x - abs(robot_next_x_coordinate))>0.02 ) or ( abs(body.worldCenter.y - abs(robot_next_y_coordinate))>0.02 ) )  ):
                        body.linearVelocity = (robot_next_x_coordinate-current_robot_position_x, robot_next_y_coordinate-current_robot_position_y)
                        self.my_world.Step(timeStep, 10, 6)
                        loop_counter+=1
                    body.linearVelocity = (0, 0)
                    body2.linearVelocity = (0, 0)

                    ##---If the robot managed to reach its destination---##
                    if(loop_counter<140 and ( abs(body.worldCenter.x-(robot_next_x_coordinate))<0.02 ) and ( abs(body.worldCenter.y-(robot_next_y_coordinate))<0.02 ) ):
                        ##---Calculate distance of robot position from the random target robot position---##
                        moveable_object_new_position = (body2.worldCenter.x, body2.worldCenter.y)
                        moveable_object_rotation = body2.angle

                        distance_x = robot_new_position[0] - robot_random_position[0]
                        distance_y = robot_new_position[1] - robot_random_position[1]
                        robot_distance = np.sqrt(np.square(distance_x) + np.square(distance_y))
                        ##---Calculate distance of moveable object from the random target object position---##
                        moveable_object_distance_x = moveable_object_new_position[0] - obstacle_random_position[0]
                        moveable_object_distance_y = moveable_object_new_position[1] - obstacle_random_position[1]
                        moveable_object_distance = np.sqrt(np.square(moveable_object_distance_x) + np.square(moveable_object_distance_y))
                        # ##---Sum up the robot distance and the moveable object distance---##
                        total_distance = round(0.4*robot_distance + 0.6*moveable_object_distance, 6)

                        ##---Store total_distance and corresponding robot coordinates and moveable object coordinates and their rotations---##
                        total_distances[total_distance] = [robot_new_position[0], robot_new_position[1], moveable_object_new_position[0], moveable_object_new_position[1], robot_rotation, moveable_object_rotation]
                        self.my_world.DestroyBody(body)
                        self.my_world.DestroyBody(body2)
                    else:
                        self.my_world.DestroyBody(body)
                        self.my_world.DestroyBody(body2)
                #print("\nLength of total_distances: ", len(total_distances))

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
                    self.my_grid[node_counter] = [closest_point[0], closest_point[1], closest_point[2], closest_point[3], robot_rotation_to_store, moveable_object_rotation_to_store]
                    #print(my_grid[node_counter])
                    node_counter += 1
                    if (node_counter % 100 == 0):
                        print(node_counter)

                #---If we are close enough to the goal---##
                if (abs(goal_position[0] - closest_point[2]) < 4.0 and abs(goal_position[1] - closest_point[3]) < 4.0):
                    found_goal_position = my_graph[node_counter-1]
                    goal_node_number = node_counter-1
                    break
            ##---If node_counter is more than the value set---##
            else:
                self.my_grid = {}
                self.my_grid[0] = [robot_start_position[0], robot_start_position[1], moveable_object_position[0], moveable_object_position[1], robot_rotation, moveable_object_rotation]
                my_graph = {}
                my_graph[0] = 0
                node_counter = 1

        self.number_of_runs += 1
        print("number_of_runs:  ", self.number_of_runs)
        print("goal_node_number: ", goal_node_number)
        # print("grid[goal_node_number: ")
        print(self.my_grid[goal_node_number])


        end_time = time.clock()

        time_needed = end_time - start_time
        print("Time needed: %f s." % time_needed)

        self.time_list.append(time_needed)
        print("Times of the runs: ")
        print(time_list)


        node_number = goal_node_number
        self.shortest_path = []
        while (True):
            self.shortest_path.append(node_number)
            previous_node_number = my_graph.get(node_number)
            node_number = previous_node_number
            if (node_number == 0):
                self.shortest_path.append(node_number)
                break

        self.shortest_path.reverse()
        #print("shortest path: ")
        print(self.shortest_path)

        ##---Create static body to mark the goal position---##
        vertices4 = [(-3.0, -1.2), (-1.2, -3.0), (1.2, -3.0), (3.0, -1.2), (3.0, 1.2), (1.2, 3.0), (-1.2, 3.0), (-3.0, 1.2)]
        body2 = self.my_world.CreateStaticBody(position=goal_position)
        # box2 = body2.CreatePolygonFixture(vertices=vertices4, density=1)
        box2 = body2.CreatePolygonFixture(box=(5.4, 5.4), density=1)
        box2.sensor = True
        body2.userData = {'color': 'goal_position'}

        ##---Create static body to mark the start position---##
        body4 = self.my_world.CreateStaticBody(position=robot_start_position)
        box4 = body4.CreatePolygonFixture(vertices=vertices2, density=1)
        box4.sensor = True
        body4.userData = {'color': 'start_position'}


        ##---Create static bodies to mark the nodes in the tree with small "dots"---##
        for key, value in self.my_grid.items():
            x = value[0]
            y = value[1]
            body3 = self.my_world.CreateStaticBody(position=(x,y))
            box3 = body3.CreatePolygonFixture(box=(0.098, 0.098), density=1)
            box3.sensor = True
            body3.userData = {'color': 'node_marker'}

        ##---Create kinematic bodies to mark the shortest path with small "dots"---##
        for number in self.shortest_path:
            x = self.my_grid[number][0]
            y = self.my_grid[number][1]
            body = self.my_world.CreateKinematicBody(position=(x, y))
            box = body.CreatePolygonFixture(box=(0.098, 0.098), density=1, friction=0.3)
            box.sensor = True
            body.userData = {'color': 'shortest_path'}


        self.colors = {
            'obstacle': (115, 115, 115),
            'robot': (165, 0, 0),
            'moveable_obstacle': (77, 166, 255),
            'start_position': (255, 140, 26),
            'goal_position': (204, 255, 153),
            'node_marker': (0, 77, 0),
            'shortest_path': (255, 128, 223),
        }


        ##---Create a new moveable object at the moveable obstacle position---##
        body = self.my_world.CreateDynamicBody(position=moveable_object_position)
        body.CreatePolygonFixture(vertices=vertices1, density=0.3, friction=1, isSensor=False, filter=Box2D.b2Filter(categoryBits=self.objectCategory, maskBits=self.objectMask))
        body.userData = {'color': 'moveable_obstacle'}

        ##---Create a dynamic body for the robot at the start position---##
        robot = self.my_world.CreateDynamicBody(position=robot_start_position, angle=0)
        robot.CreatePolygonFixture(vertices=vertices2, density=1, friction=0.3, isSensor=False, filter=Box2D.b2Filter(categoryBits=self.robotCategory, maskBits=self.robotMask))
        robot.CreatePolygonFixture(vertices=horizontal_arm_base, density=1, friction=1, isSensor=False)
        robot.CreatePolygonFixture(vertices=left_long_arm, density=1, friction=1, isSensor=False)
        robot.CreatePolygonFixture(vertices=right_long_arm, density=1, friction=1, isSensor=False)
        robot.CreatePolygonFixture(vertices=left_short_arm, density=1, friction=1, isSensor=False)
        robot.CreatePolygonFixture(vertices=right_short_arm, density=1, friction=1, isSensor=False)
        robot.userData = {'color': 'robot'}

        # end_time = datetime.now()
        # total_runtime = end_time - start_time
        # print("total_runtime: %s" % (total_runtime))

        # ##---Create a dynamic body for the robot at the goal position---####### ONLY FOR IMAGES IN REPORT!
        # dynamic_body = my_world.CreateDynamicBody(position=goal_position, angle=0)
        # box = dynamic_body.CreatePolygonFixture(vertices=vertices2, density=1, friction=0.3)




##---main game loop---##
rob_x = 36
rob_y = 3
rob_rot = 0
obs_x = 46
obs_y = 27
obs_rot = 0
number_of_runs = 0
time_list = []

while (True):
    my_world = World(rob_x, rob_y, rob_rot, obs_x, obs_y, obs_rot, number_of_runs, time_list)
    #print("Number of bodies in the world 1: ", len(my_world.my_world.bodies))

    grid = my_world.my_grid

    running = True
    goal_reached = False        ## Variable to store value if the goal state is reached or not
    obstacle_reached = False    ## Variable to store value if the robot has already reached the obstacle or not
    obstacle_lost = False       ## Variable to store value if the robot has lost the obstacle after reaching that
    counter = 0
    target_coordinate_list = []

    ## Get shortest path
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
    printcounter = 0
    ###print(grid)
    while (running):
        # Check the event queue
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                running = False


        my_world.screen.fill((26, 26, 26, 0))
        robot = my_world.my_world.bodies[-1]
        moveable_obstacle = my_world.my_world.bodies[-2]

        # Draw the world
        for body in (my_world.my_world.bodies):  # or: world.bodies
            for fixture in body.fixtures:
                shape = fixture.shape
                vertices = [(body.transform * v) * my_world.PPM for v in shape.vertices]
                vertices = [(v[0], my_world.SCREEN_HEIGHT - v[1]) for v in vertices]

                #pygame.draw.polygon(my_world.screen, my_world.colors[body.type], vertices)
                pygame.draw.polygon(my_world.screen, my_world.colors[body.userData['color']], vertices)
        my_world.my_world.Step(my_world.TIME_STEP, 6, 2)
        #print("Number of bodies in the world: ", len(my_world.my_world.bodies))
        pygame.display.flip()
        my_world.clock.tick(my_world.TARGET_FPS)


        ##---The target is the actual element of the target coordinate list---##
        target = target_coordinate_list[counter]
        if(counter < len(target_coordinate_list)-1):
            next_angle = target_coordinate_list[counter][4]
        else:
            next_angle = target_coordinate_list[counter][4]


        if (goal_reached == False):
            current_angle = robot.angle % (2*math.pi)
            angle_to_turn = (next_angle - current_angle) % (2*math.pi)

            if (angle_to_turn < 0.03 and angle_to_turn > -0.03):
                correct_angle = True
            else:
                correct_angle = False

            if (correct_angle == False):
                if ((angle_to_turn>0 and angle_to_turn<math.pi) or (angle_to_turn>-2*math.pi and angle_to_turn< -math.pi)):
                    #robot.angle += math.pi/320
                    robot.angularVelocity = (0.50)
                    #my_world.my_world.Step(my_world.TIME_STEP, 6, 2)
                else:
                    #robot.angle -= math.pi / 320
                    robot.angularVelocity = (-0.50)
                    #my_world.my_world.Step(my_world.TIME_STEP, 6, 2)

            #correct_angle=True
            if (correct_angle == True):
                ##---Calculate x, y distances of the actual target and the robot body and set velocity to move towards the target---##
                if (np.square(target[0]-robot.worldCenter.x) > 0.001 or np.square(target[1]-robot.worldCenter.y) > 0.001):
                    direction_x = target[0] - robot.worldCenter.x
                    direction_y = target[1] - robot.worldCenter.y
                    distance = np.sqrt(np.square(direction_x)+np.square(direction_y))
                    speed = 1.9 * 1/distance
                    robot_rotation = target[4]
                    robot.linearVelocity = (speed*direction_x, speed*direction_y)
                    moveable_obstacle.linearVelocity = (0, 0)
                    moveable_obstacle.angularVelocity = (0)
                    my_world.my_world.Step(my_world.TIME_STEP, 6, 2)

                    ##---Calculate obstacle's distance from the robot---##
                    distance_x = moveable_obstacle.worldCenter.x - robot.worldCenter.x
                    distance_y = moveable_obstacle.worldCenter.y - robot.worldCenter.y
                    distance = np.sqrt(np.square(distance_x)+np.square(distance_y))
                    if(distance < 2.0): obstacle_reached = True

                ##---If robot reaches the actual target point---##
                else:
                    # print("counter:                ", targetlist[counter])
                    # print("robot coordinates:      ", robot.worldCenter.x, robot.worldCenter.y)
                    # print("my_grid robot coord:    ", target[0], target[1])
                    # print("obstacle coordinates:   ", moveable_obstacle.worldCenter.x, moveable_obstacle.worldCenter.y)
                    # print("my_grid obstacle coord: ", target[2], target[3])
                    # print("x difference:           ", moveable_obstacle.worldCenter.x-target[2])
                    # print("y difference:           ", moveable_obstacle.worldCenter.y - target[3])
                    print("---------")

                    # moveable_obstacle.worldCenter.x = target[2]
                    # moveable_obstacle.worldCenter.y = target[3]
                    counter += 1

                ##---If robot reaches the final goal node (within a certain threshold)---##
                if (np.square(target_coordinate_list[goal_node][0] - robot.worldCenter.x) < 0.01
                    and np.square(target_coordinate_list[goal_node][1] - robot.worldCenter.y) < 0.01):
                    goal_reached=True

                robot.linearVelocity = (0, 0)
                robot.angularVelocity = (0)
                moveable_obstacle.linearVelocity = (0, 0)
                moveable_obstacle.angularVelocity = (0)

                ##---Check if the robot lost the obstacle or not---##
                if(obstacle_reached == True and distance > 4.0):
                    print("obstacle lost")
                    obstacle_reached = False
                    obstacle_lost = True
                    rob_x = robot.worldCenter.x
                    rob_y = robot.worldCenter.y
                    rob_rot = robot.angle
                    obs_x = moveable_obstacle.worldCenter.x
                    obs_y = moveable_obstacle.worldCenter.y
                    obs_rot = 0
                    number_of_runs = my_world.number_of_runs
                    time_list = my_world.time_list
                    break


            if (goal_reached == True and printcounter == 0):
                print("obstacle x, y: ", round(moveable_obstacle.worldCenter.x, 2), round(moveable_obstacle.worldCenter.y, 2))
                print("goal position: 15, 14")
                printcounter += 1




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
    #

    pygame.quit()
    print('Done!')


#    source activate py34
