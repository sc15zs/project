#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math, time, datetime
from datetime import datetime
import numpy as np
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)

import Box2D
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, kinematicBody)



def dijkstra(graph, src, dest, counter, visited=[], distances={}, predecessors={}):
    start = time.time()
    ##---Check if the start node and the goal node are in the graph---##
    if src not in graph:
        raise TypeError('The root of the shortest path tree cannot be found')
    if dest not in graph:
        raise TypeError('The target of the shortest path cannot be found')
    #     # ending condition
    if src == dest:
        # We build the shortest path and display it
        path = []
        pred = dest
        while pred != None:
            path.append(pred)
            pred = predecessors.get(pred, None)
        end = time.time()
       # print("Time needed for planning: %f" % end-start)
        #print('shortest path: ' + str(path) + " cost=" + str(distances[dest]))
        return path, distances[dest], counter
    else:
        # if it is the initial  run, initializes the cost
        if not visited:
            distances[src] = 0
        # visit the neighbors
        for neighbor in graph[src]:
            if neighbor not in visited:
                new_distance = distances[src] + graph[src][neighbor]
                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    predecessors[neighbor] = src
        # mark as visited
        visited.append(src)
        # now that all neighbors have been visited: recurse
        # select the non visited node with lowest distance 'x'
        # run Dijskstra with src='x'
        unvisited = {}
        for k in graph:
            if k not in visited:
                unvisited[k] = distances.get(k, float('inf'))
        x = min(unvisited, key=unvisited.get)
        counter += 1
        return dijkstra(graph, x, dest, counter, visited, distances, predecessors)


def A_star(grid, graph, src, dest, counter, visited=[], distances={}, predecessors={}, cost_plus_euclidean_distances={}):
    if src not in graph:
        raise TypeError('The root of the shortest path tree cannot be found')
    if dest not in graph:
        raise TypeError('The target of the shortest path cannot be found')
    dest_x_coordinate = grid[dest][0]
    dest_y_coordinate = grid[dest][1]
        #     # ending condition
    if src == dest:
        # We build the shortest path and display it
        path = []
        pred = dest
        while pred != None:
            path.append(pred)
            pred = predecessors.get(pred, None)
        #print('shortest path: ' + str(path) + " cost=" + str(distances[dest]))
        return path, distances[dest], counter
    else:
        # if it is the initial  run, initializes the cost
        if not visited:
            distances[src] = 0
            cost_plus_euclidean_distances[src] = 0
        # visit the neighbors
        for neighbor in graph[src]:
            if neighbor not in visited:
                new_distance = distances[src] + graph[src][neighbor]

                # calculate euclidean distance
                distance_x = dest_x_coordinate - grid[neighbor][0]
                distance_y = dest_y_coordinate - grid[neighbor][1]
                euclidean_distance = np.sqrt(np.square(distance_x) + np.square(distance_y))

                total_distance = new_distance + euclidean_distance
                cost_plus_euclidean_distances[neighbor] = total_distance

                if new_distance < distances.get(neighbor, float('inf')):
                    distances[neighbor] = new_distance
                    predecessors[neighbor] = src
        # mark as visited
        visited.append(src)
        # now that all neighbors have been visited: recurse
        # select the non visited node with lowest distance 'x'
        # run A_star with src='x'
        unvisited = {}
        for k in cost_plus_euclidean_distances:
            if k not in visited:
                unvisited[k] = cost_plus_euclidean_distances.get(k, float('inf'))

        x = min(unvisited, key=unvisited.get)
        counter+=1
        return A_star(grid, graph, x, dest, counter, visited, distances, predecessors, cost_plus_euclidean_distances)


class World():

    def create_world_grid():
        # CREATE DICTIONARY FOR STORING WORLD CELLS
        my_world = {}
        key = 0
        for x in range(600):
            my_world[key] = [-100, -100]
            key += 1
        # STORE x AND y COORDINATES FOR EACH CELL
        key = 0
        for y in xrange(1, 40, 2):
            for x in xrange(1, 60, 2):
                my_world[key] = [x, y]
                key += 1
        return my_world


    def create_graph():
        # BUILD A FULLY CONNECTED GRAPH OF THE WORLD: AT THE BEGINNING EVERY CELL IS
        # CONSIDERED TO BE FREE
        my_graph = {}
        for key in range(600):
            my_graph[key] = {key + 30: 1, key + 31: 1.4, key + 1: 1, key - 29: 1.4, key - 30: 1, key - 31: 1.4,
                             key - 1: 1, key + 29: 1.4}

        # Define list of cells for each side (corner cells not included!)
        left_side = [30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330, 360, 390, 420, 450, 480, 510, 540]
        right_side = [59, 89, 119, 149, 179, 209, 239, 269, 299, 329, 359, 389, 419, 449, 479, 509, 539, 569]
        bottom_side = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                       25, 26, 27, 28]
        top_side = [571, 572, 573, 574, 575, 576, 577, 578, 579, 580, 581, 582, 583, 584, 585, 586, 587, 588,
                    589, 590, 591, 592, 593, 594, 595, 596, 597, 598]
        for key in range(600):
            # set connected cells of side cells (corner cells not included):
            if (key in left_side):
                my_graph[key] = {key + 30: 1, key + 31: 1.4, key + 1: 1, key - 29: 1.4, key - 30: 1}
            if (key in right_side):
                my_graph[key] = {key + 30: 1, key - 30: 1, key - 31: 1.4, key - 1: 1, key + 29: 1.4}
            if (key in bottom_side):
                my_graph[key] = {key + 30: 1, key + 31: 1.4, key + 1: 1, key - 1: 1, key + 29: 1.4}
            if (key in top_side):
                my_graph[key] = {key + 1: 1, key - 29: 1.4, key - 30: 1, key - 31: 1.4, key - 1: 1}
            # set connections of corner cells:
            if (key == 0):
                my_graph[key] = {key + 30: 1, key + 31: 1.4, key + 1: 1}
            if (key == 29):
                my_graph[key] = {key + 30: 1, key - 1: 1, key + 29: 1.4}
            if (key == 570):
                my_graph[key] = {key + 1: 1, key - 29: 1.4, key - 30: 1}
            if (key == 599):
                my_graph[key] = {key - 30: 1, key - 31: 1.4, key - 1: 1}
        return my_graph



    # --- constants ---
    # Box2D deals with meters, but we want to display pixels,
    # so define a conversion factor:
    PPM = 20.0  # pixels per meter
    TARGET_FPS = 60
    TIME_STEP = 1.0 / TARGET_FPS
    SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 800

    # --- pygame setup ---
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    pygame.display.set_caption('Path finding: robot body of any shape')
    clock = pygame.time.Clock()
    start_time = datetime.now()

    # --- pybox2d world setup ---
    # Create the world
    my_world = world(gravity=(0, 0), doSleep=True)

    # ##Create obstacles
    # wall_bottom = my_world.CreateStaticBody(position=(30, 0.1), shapes=polygonShape(box=(30, 0.1)), )
    # wall_top = my_world.CreateStaticBody(position=(30, 40), shapes=polygonShape(box=(30, 0.1)), )
    # wall_left = my_world.CreateStaticBody(position=(0, 20), shapes=polygonShape(box=(20, 0.1)), angle=math.pi/2)
    # wall_right = my_world.CreateStaticBody(position=(60, 20), shapes=polygonShape(box=(20, 0.1)), angle=math.pi/2)
    # wall_bottom.userData = {'color': 'obstacle'}
    # wall_top.userData = {'color': 'obstacle'}
    # wall_left.userData = {'color': 'obstacle'}
    # wall_right.userData = {'color': 'obstacle'}

    obstacle_1 = my_world.CreateStaticBody(position=(5, 12.5), shapes=polygonShape(box=(5, 2.5)))
    obstacle_2 = my_world.CreateStaticBody(position=(40, 25), shapes=polygonShape(box=(6, 2)), angle=math.pi / 4)
    obstacle_3 = my_world.CreateStaticBody(position=(30, 25), shapes=polygonShape(box=(6, 2)), angle=-math.pi / 4)
    obstacle_1.userData = {'color': 'obstacle'}
    obstacle_2.userData = {'color': 'obstacle'}
    obstacle_3.userData = {'color': 'obstacle'}

    # ### TO BE USED TO TEST DIJKSTRA'S AND A*
    # obstacle_1 = my_world.CreateStaticBody(position=(12, 12), shapes=polygonShape(box=(8, 0.5)), angle=-math.pi/4)
    # obstacle_1.userData = {'color': 'obstacle'}
    # obstacle_2 = my_world.CreateStaticBody(position=(24, 8), shapes=polygonShape(box=(8, 0.5)), angle=-math.pi/4)
    # obstacle_2.userData = {'color': 'obstacle'}
    # obstacle_3 = my_world.CreateStaticBody(position=(18, 20), shapes=polygonShape(box=(8, 0.5)), angle=-math.pi/4)
    # obstacle_3.userData = {'color': 'obstacle'}
    # obstacle_4 = my_world.CreateStaticBody(position=(22, 31), shapes=polygonShape(box=(14, 1.0)), angle=-math.pi/4)
    # obstacle_4.userData = {'color': 'obstacle'}
    # obstacle_5 = my_world.CreateStaticBody(position=(37, 25), shapes=polygonShape(box=(12, 1.0)), angle=-math.pi/2)
    # obstacle_5.userData = {'color': 'obstacle'}
    # obstacle_6 = my_world.CreateStaticBody(position=(44, 14), shapes=polygonShape(box=(6, 3.0)), angle=-math.pi/4)
    # obstacle_6.userData = {'color': 'obstacle'}
    # obstacle_7 = my_world.CreateStaticBody(position=(54, 26), shapes=polygonShape(box=(10, 1.0)))
    # obstacle_7.userData = {'color': 'obstacle'}



    # dynamic_body2 = my_world.CreateDynamicBody(position=(31, 8), angle=0 * math.pi / 2)
    # # And add a box fixture onto it (with a nonzero density, so it will move)
    # box2 = dynamic_body2.CreatePolygonFixture(box=(1.38, 1.78), density=5, friction=0.3)



    my_grid = create_world_grid()
    my_graph = create_graph()

    # CHECK IF CELLS ARE FREE OR OCCUPIED
    timeStep = 1.0 / 60
    vel_iters, pos_iters = 6, 2
    key = 0
    occupied_counter = 0
    occupied_list = []
    free_cell_list = []
    for y in xrange(1, 40, 2):
        for x in xrange(1, 60, 2):
            start_position = (x, y)
            body = my_world.CreateDynamicBody(position=start_position)
            box = body.CreatePolygonFixture(box=(0.98, 1.98), density=1, friction=0.3)
            free_cell_list.append(key)
            for i in range(3):
                # Instruct the world to perform a single step of simulation. It is
                my_world.Step(timeStep, vel_iters, pos_iters)
            #print (start_position, body.position)
            if (start_position != body.position):
                occupied_counter += 1
                occupied_list.append(key)
                my_grid[key] = [1000, 1000]  # these x and y values will denote that the cell is occupied
                del my_graph[key]
                free_cell_list.remove(key)

            my_world.DestroyBody(body)
            key += 1

    # # # Uncomment if want to draw not occupied cells
    # for item, coordinates in my_grid.items():
    #     if item in free_cell_list:
    #         x = coordinates[0]
    #         y = coordinates[1]
    #         body2 = my_world.CreateStaticBody(position=(x,y))
    #         box = body2.CreatePolygonFixture(box=(0.098, 0.098), density=1, friction=0.3)


    #print ("occupied_counter: %d" % occupied_counter)

    # count number of cells in the graph
    counter = 0
    for key, value in my_graph.items():
        counter += 1
    # print("Number of free cells in graph: %d" % counter)
    # print("occupied cells: %s" % occupied_list)

    # count edges in graph so far
    edge_count = 0
    for key, value in my_graph.items():
        for x in value:
            edge_count += 1
    #print("edgecount: %d" % edge_count)

    # delete edges which are connected to occupied cells
    for number in occupied_list:
        for key, value in my_graph.items():
            for x, y in value.items():
                if (x == number):
                    del value[x]

    ##---Delete diagonal edges at the corners of occupied cells---##
    ##---Those edges which connect two free cell diagonally but there is an occupied cells next to them---##
    for number in occupied_list:
        ##---Delete edges at the top-right corner of the occupied cell---##
        if (number+1 in my_graph.keys() and number+30 in my_graph.keys()):
            if (number+30 in my_graph[number+1].keys()):
                del my_graph[number+1][number+30]
            if (number+1 in my_graph[number+30].keys()):
                del my_graph[number+30][number+1]
        ##---Delete edges at the top-left corner of the occupied cell---##
        if (number-1 in my_graph.keys() and number+30 in my_graph.keys()):
            if (number+30 in my_graph[number-1].keys()):
                del my_graph[number-1][number+30]
            if (number-1 in my_graph[number+30].keys()):
                del my_graph[number+30][number-1]
        ##---Delete edges at the bottom-left corner of the occupied cell---##
        if (number-1 in my_graph.keys() and number-30 in my_graph.keys()):
            if (number-30 in my_graph[number-1].keys()):
                del my_graph[number-1][number-30]
            if (number-1 in my_graph[number-30].keys()):
                del my_graph[number-30][number-1]
        ##---Delete edges at the bottom-right corner of the occupied cell---##
        if (number+1 in my_graph.keys() and number-30 in my_graph.keys()):
            if (number-30 in my_graph[number+1].keys()):
                del my_graph[number+1][number-30]
            if (number+1 in my_graph[number-30].keys()):
                del my_graph[number-30][number+1]


    start_key = 0
    for y in xrange(1, 40, 2):
        for x in xrange(1, 60, 2):
            start_position3 = (x, y)
            for i in xrange(0, 39, 1):
                body3 = my_world.CreateDynamicBody(position=start_position3, angle=i * math.pi / 20)  # step size: 9 degrees
                box = body3.CreatePolygonFixture(box=(0.98, 1.98), density=1, friction=0.3)
                for j in range(3):
                    # Instruct the world to perform a single step of simulation.
                    my_world.Step(timeStep, vel_iters, pos_iters)

                if (start_position3 != body3.position):
                    if (i < 35):  # 35*9=315 degrees
                        for key, value in my_graph.items():
                            if (key == start_key):
                                for inner_key, inner_value in value.items():
                                    if (key + 31 == inner_key):
                                        del value[key + 31]
                    if (i < 30):  # 30*9=270 degrees
                        for key, value in my_graph.items():
                            if (key == start_key):
                                for inner_key, inner_value in value.items():
                                    if (key + 1 == inner_key):
                                        del value[key + 1]
                    if (i < 25):  # 25*9=225 degrees
                        for key, value in my_graph.items():
                            if (key == start_key):
                                for inner_key, inner_value in value.items():
                                    if (key - 29 == inner_key):
                                        del value[key - 29]
                    if (i < 20):  # 20*9=180 degrees
                        for key, value in my_graph.items():
                            if (key == start_key):
                                for inner_key, inner_value in value.items():
                                    if (key - 30 == inner_key):
                                        del value[key - 30]
                    if (i < 15):  # 15*9=135 degrees
                        for key, value in my_graph.items():
                            if (key == start_key):
                                for inner_key, inner_value in value.items():
                                    if (key - 31 == inner_key):
                                        del value[key - 31]
                    if (i < 10):  # 10*9=00 degrees
                        for key, value in my_graph.items():
                            if (key == start_key):
                                for inner_key, inner_value in value.items():
                                    if (key - 1 == inner_key):
                                        del value[key - 1]
                    if (i < 5):  # 5*9=45 degrees
                        for key, value in my_graph.items():
                            if (key == start_key):
                                for inner_key, inner_value in value.items():
                                    if (key + 29 == inner_key):
                                        del value[key + 29]
                my_world.DestroyBody(body3)
            start_key += 1


    # count edges again
    new_edge_count = 0
    for key, value in my_graph.items():
        #print(key, value)
        for x in value:
            new_edge_count += 1
    #print("new edgecount: %d" % new_edge_count)

    # goal_number = 407
    # start_time_dijkstra = datetime.now()
    # shortest_path_1, cost_1, counter_1 = dijkstra(my_graph,31, goal_number, 0)
    # shortest_path_1.reverse()
    # #print("Shortest path using Dijkstra's algorithm: ")
    # print(shortest_path_1)
    # print("cost: %f" % cost_1)
    # print("Number of nodes expanded: %d" % counter_1)
    # end_time_dijkstra = datetime.now()
    # time_needed_dijkstra = end_time_dijkstra - start_time_dijkstra
    # print("Time needed for Dijkstra's algorithm: %f s." % time_needed_dijkstra)
    #
    # start_time_A_star = datetime.now()
    # shortest_path_2, cost_2, counter_2 = A_star(my_grid, my_graph,31, goal_number, 0)
    # shortest_path_2.reverse()
    # #print("Shortest path using A* algorithm: ")
    # print(shortest_path_2)
    # print("cost: %f" % cost_2)
    # print("Number of nodes expanded: %d" % counter_2)
    # end_time_A_star = datetime.now()
    # time_needed_A_star = end_time_A_star - start_time_A_star
    # print("Time needed for A* algorithm: %f s." % time_needed_A_star)

    ###TO TEST DIJKSTRA'S AND A*
    start_number = 31
    goal_number = 437
    start_time_dijkstra = datetime.now()
    shortest_path_1, cost_1, counter_1 = dijkstra(my_graph, start_number, goal_number, 0)
    shortest_path_1.reverse()
    #print("Shortest path using Dijkstra's algorithm: ")
    print(shortest_path_1)
    print("cost: %f" % cost_1)
    print("Number of nodes expanded: %d" % counter_1)
    end_time_dijkstra = datetime.now()
    time_needed_dijkstra = end_time_dijkstra - start_time_dijkstra
    print("Time needed for Dijkstra's algorithm: %s s." % str(time_needed_dijkstra))
    # print("%s" % str(start_time_dijkstra))
    # print("%s" % str(end_time_dijkstra))

    start_time_A_star = datetime.now()
    shortest_path_2, cost_2, counter_2 = A_star(my_grid, my_graph, start_number, goal_number, 0)
    shortest_path_2.reverse()
    #print("Shortest path using A* algorithm: ")
    print(shortest_path_2)
    print("cost: %f" % cost_2)
    print("Number of nodes expanded: %d" % counter_2)
    end_time_A_star = datetime.now()
    time_needed_A_star = end_time_A_star - start_time_A_star
    print("Time needed for A* algorithm: %s s." % str(time_needed_A_star))
    # print("%s" % str(start_time_A_star))
    # print("%s" % str(end_time_A_star))


    ##---Create static bodies to mark the nodes in the tree with small "dots"---##
    for key, value in my_grid.items():
        x = value[0]
        y = value[1]
        body3 = my_world.CreateStaticBody(position=(x, y))
        box3 = body3.CreatePolygonFixture(box=(0.098, 0.098), density=1)
        box3.sensor = True
        body3.userData = {'color': 'node_marker'}

    # # Draw small circles along the shortest path
    for number in shortest_path_1:
        x = my_grid[number][0]
        y = my_grid[number][1]
        body = my_world.CreateKinematicBody(position=(x,y))
        box = body.CreatePolygonFixture(box=(0.098, 0.098), density=1, friction=0.3)
        box.sensor = True
        body.userData = {'color': 'shortest_path'}


    vertices2 = [(-0.5, -0.2), (-0.2, -0.5), (0.2, -0.5), (0.5, -0.2), (0.5, 0.2), (0.2, 0.5), (-0.2, 0.5), (-0.5, 0.2)]
    ##---Create static body to mark the start position---##
    # body1 = my_world.CreateStaticBody(position=(31, 3))
    body1 = my_world.CreateStaticBody(position= (my_grid[start_number][0], my_grid[start_number][1]))
    box1 = body1.CreatePolygonFixture(vertices=vertices2, density=1)
    box1.sensor = True
    body1.userData = {'color': 'start_position'}
    ##---Create static body to mark the goal position---##
    body2 = my_world.CreateStaticBody(position=(my_grid[goal_number][0], my_grid[goal_number][1]))
    box2 = body2.CreatePolygonFixture(vertices=vertices2, density=1)
    box2.sensor = True
    body2.userData = {'color': 'goal_position'}

    # Create a dynamic body for the robot
    # dynamic_body = my_world.CreateDynamicBody(position=(31, 3), angle=0*math.pi/2)
    dynamic_body = my_world.CreateDynamicBody(position=(my_grid[start_number][0], my_grid[start_number][1]), angle=0*math.pi/2)
    # And add a box fixture onto it (with a nonzero density, so it will move)
    box = dynamic_body.CreatePolygonFixture(box=(0.5, 0.8), density=1, friction=0.3)
    dynamic_body.userData = {'color': 'robot'}

    colors = {
        'obstacle': (115, 115, 115),
        'robot': (165, 0, 0),
        'moveable_obstacle': (77, 166, 255),
        'start_position': (255, 140, 26),
        'goal_position': (204, 255, 153),
        'node_marker': (0, 77, 0),
        'shortest_path': (255, 128, 223),
    }
    end_time = datetime.now()
    total_runtime = end_time - start_time
    print("total_runtime: %s" % (total_runtime))



my_world = World()
# --- main game loop ---
running = True
counter = 0
goal_reached = False
grid = my_world.my_grid
target_coordinate_list = []
correct_angle = False

# Get shortest path
targetlist = my_world.shortest_path_1         ##  TO BE USED FOR DIJKSTRA'S
#targetlist = my_world.shortest_path_2         ##  TO BE USED FOR A*
## Create list of the shortest path cells: for each cell get the x coordinate, y coordinate and cell number
for n in range(len(targetlist)):
    list = grid[targetlist[n]]
    list.append(targetlist[n])
    target_coordinate_list.append(list)
# print("targets:::::::::::")
#print ("\n\ntarget_coordinate_list: ", target_coordinate_list)

##print(grid)
while running:
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

    my_world.screen.fill((0, 0, 0, 0))
    robot = my_world.my_world.bodies[-1]
    # print ("position:")
    # print(robot.worldCenter.x, robot.worldCenter.y)

    # Draw the world
    for body in (my_world.my_world.bodies):  # or: world.bodies
        # The body gives us the position and angle of its shapes
        for fixture in body.fixtures:
            # The fixture holds information like density and friction, and also the shape.
            shape = fixture.shape

            # Naively assume that this is a polygon shape. (not good normally!)
            # We take the body's transform and multiply it with each
            # vertex, and then convert from meters to pixels with the scale
            # factor.
            vertices = [(body.transform * v) * my_world.PPM for v in shape.vertices]

            # But wait! It's upside-down! Pygame and Box2D orient their
            # axes in different ways. Box2D is just like how you learned
            # in high school, with positive x and y directions going
            # right and up. Pygame, on the other hand, increases in the
            # right and downward directions. This means we must flip
            # the y components.
            vertices = [(v[0], my_world.SCREEN_HEIGHT - v[1]) for v in vertices]

            #pygame.draw.polygon(my_world.screen, my_world.colors[body.type], vertices)
            pygame.draw.polygon(my_world.screen, my_world.colors[body.userData['color']], vertices)

    # Make Box2D simulate the physics of our world for one step.
    # Instruct the world to perform a single step of simulation. It is
    # generally best to keep the time step and iterations fixed.
    # See the manual (Section "Simulating the World") for further discussion
    # on these parameters and their implications.
    my_world.my_world.Step(my_world.TIME_STEP, 10, 10)
    # print(len(my_world.my_world.bodies))

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    my_world.clock.tick(my_world.TARGET_FPS)

    if (goal_reached==False):
        target = target_coordinate_list[counter]
    #print ("target: " + str(target))

    if (counter == 0):
        if (target_coordinate_list[counter + 1][2] - 30 == target_coordinate_list[counter][2]):  next_angle = 0
        if (target_coordinate_list[counter + 1][2] - 29 == target_coordinate_list[counter][2]):  next_angle = math.pi/4
        if (target_coordinate_list[counter + 1][2] +  1 == target_coordinate_list[counter][2]):  next_angle = math.pi/2
        if (target_coordinate_list[counter + 1][2] + 31 == target_coordinate_list[counter][2]):  next_angle = 3 * math.pi/4
        if (target_coordinate_list[counter + 1][2] + 30 == target_coordinate_list[counter][2]):  next_angle = math.pi
        if (target_coordinate_list[counter + 1][2] + 29 == target_coordinate_list[counter][2]):  next_angle = 5 * math.pi/4
        if (target_coordinate_list[counter + 1][2] -  1 == target_coordinate_list[counter][2]):  next_angle = 3 * math.pi/2
        if (target_coordinate_list[counter + 1][2] - 31 == target_coordinate_list[counter][2]):  next_angle = 7 * math.pi/4

    # Determine next angle
    if (counter > 0):
        if (target_coordinate_list[counter][2] - 30 == target_coordinate_list[counter-1][2]):  next_angle = 0
        elif (target_coordinate_list[counter][2] - 29 == target_coordinate_list[counter-1][2]):  next_angle = math.pi/4
        elif (target_coordinate_list[counter][2] +  1 == target_coordinate_list[counter-1][2]):  next_angle = math.pi/2
        elif (target_coordinate_list[counter][2] + 31 == target_coordinate_list[counter-1][2]):  next_angle = 3*math.pi/4
        elif (target_coordinate_list[counter][2] + 30 == target_coordinate_list[counter-1][2]):  next_angle = math.pi
        elif (target_coordinate_list[counter][2] + 29 == target_coordinate_list[counter-1][2]):  next_angle = 5*math.pi/4
        elif (target_coordinate_list[counter][2] -  1 == target_coordinate_list[counter-1][2]):  next_angle = 3*math.pi/2
        elif (target_coordinate_list[counter][2] - 31 == target_coordinate_list[counter-1][2]):  next_angle = 7*math.pi/4


    if (goal_reached == False):
        current_angle = robot.angle % (2*math.pi)
        angle_to_turn = next_angle - current_angle

        if (angle_to_turn < 0.03 and angle_to_turn > -0.03):
            correct_angle = True
        else:
            correct_angle = False

        if (correct_angle == False):
            if ((angle_to_turn > 0 and angle_to_turn < math.pi) or (angle_to_turn > -2*math.pi and angle_to_turn < -math.pi)):
                robot.angle += math.pi/160
            else:
                robot.angle -= math.pi / 160

        if (correct_angle == True):
            if (np.square(target[0] - robot.worldCenter.x) > 0.01 or np.square(target[1] - robot.worldCenter.y) > 0.01):
                direction_x = target[0] - robot.worldCenter.x
                direction_y = target[1] - robot.worldCenter.y
                distance = np.sqrt(np.square(direction_x)+np.square(direction_y))
                speed = 1.9 * 1/distance

                robot.linearVelocity = (speed*direction_x, speed*direction_y)
                my_world.my_world.Step(my_world.TIME_STEP, 10, 10)
                #my_world.clock.tick(30)
            else:
                if (counter < len(targetlist)-1):
                    counter+=1
                else:
                    goal_reached = True
            my_world.dynamic_body.linearVelocity = (0, 0)




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


pygame.quit()
print('Done!')
