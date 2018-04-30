#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math, time
from datetime import datetime
import numpy as np
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import random

import Box2D  # The main library
# Box2D.b2 maps Box2D.b2Vec2 to vec2 (and so on)
from Box2D.b2 import (world, polygonShape, staticBody, dynamicBody, kinematicBody)


class World():

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

    def calculate_distance (x, y, target_x, target_y):
        return np.sqrt(np.square(abs(x) - abs(target_x)) + np.square(abs(y) - abs(target_y)))

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
    SCREEN_WIDTH, SCREEN_HEIGHT = 1200, 800

    ##---pygame setup---##
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    pygame.display.set_caption('Path finding: robot body of any shape')
    clock = pygame.time.Clock()
    start_time = datetime.now()

    # Create the world
    my_world = world(gravity=(0, 0), doSleep=True)

    # ##---Create static bodies for walls and obstacles---##
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

    ##---Vertices for the robot body---##
    vertices2 = [(-0.5, -0.2), (-0.2, -0.5), (0.2, -0.5), (0.5, -0.2), (0.5, 0.2), (0.2, 0.5), (-0.2, 0.5), (-0.5, 0.2)]

    ##---Dictionary to hold node numbers as keys and neighbouring node numbers and their distances as values---##
    my_graph = {}
    ##---Dictionary to hold node numbers as keys and x,y coordinates as values---##
    my_grid = {}
    timeStep = 1.0 / 60
    vel_iters, pos_iters = 6, 2
    #start_position = (31, 3)
    robot_start_position = (1, 1)
    goal_position = (35.0, 25.0)
    #goal_position = (5.0, 5.0)

    my_grid[0] = [robot_start_position[0], robot_start_position[1]]
    my_graph[0] = {0: 0}
    node_counter = 1
    while (True):
        if (node_counter < 2500):
            if (node_counter % 8 == 0):
                ##---The goal position is the "random" point---##
                x = goal_position[0]
                y = goal_position[1]
            else:
                ##---Generate random x and y coordinates and round them to two decimal places---##
                x = round(random.uniform(0.0, 60.0), 2)
                y = round(random.uniform(0.0, 40.0), 2)
            random_position = (x, y)
            ##---Find closest node in graph to the random point---##
            closest_node_number = find_closest_node(my_grid, x, y)

            ##---Get the 8 possible movements from this node---##
            coordinates = calculate_coordinates(my_grid[closest_node_number][0], my_grid[closest_node_number][1], x, y)
           # print(coordinates)
            ##--- Check the possible moves for collision one by one: starting with the point closest to the random point---##
            for n in sorted(coordinates):
                x_coordinate = coordinates[n][0]
                y_coordinate = coordinates[n][1]
                robot_new_position = (x_coordinate, y_coordinate)
                body = my_world.CreateDynamicBody(position=robot_new_position)
                box = body.CreatePolygonFixture(vertices=vertices2, density=1)
                #print("here")
                for i in range(3):
                    # Instruct the world to perform a single step of simulation.
                    my_world.Step(timeStep, vel_iters, pos_iters)
                ##---If robot body is not in collision, add point to the graph and break from for loop---##
                if (robot_new_position == body.position):
                    ##---Calculate distance of robot position from the previous (closest) node---##
                    distance_x = robot_new_position[0] - my_grid[closest_node_number][0]
                    distance_y = robot_new_position[1] - my_grid[closest_node_number][1]
                    distance = round(np.sqrt(np.square(distance_x) + np.square(distance_y)), 1)
                    # ##---Store in graph the node number, the neighbour node and its distance---##
                    # ##---For both the new node and to the previous (closest) node---##
                    # my_graph[node_counter] = {closest_node_number: distance}
                    # value = my_graph.get(closest_node_number)
                    # value[node_counter] = distance
                    # my_graph[closest_node_number] = value
                    # ##---Store new node number and its x, y coordinates---##
                    # my_grid[node_counter] = [robot_new_position[0], robot_new_position[1]]

                    ##---Store in the graph the previous (parent) node---##
                    my_graph[node_counter] = closest_node_number

                    ##---Store new node number and its x, y coordinates---##
                    my_grid[node_counter] = [robot_new_position[0], robot_new_position[1]]


                    node_counter += 1
                    if (node_counter % 100 == 0):
                        print(node_counter)
                    my_world.DestroyBody(body)
                    break
                ##---If robot body is in collision, stay in the loop and check the next possible move---##
                else:
                    my_world.DestroyBody(body)
            ##---If we are close enough to the goal---##
            if ((np.square(goal_position[0] - body.position.x) < 0.5) and (np.square(goal_position[1] - body.position.y) < 0.5)):
                found_goal_position = my_graph[node_counter-1]
                goal_node_number = node_counter-1
                break
        else:
            my_grid = {}
            my_grid[0] = [robot_start_position[0], robot_start_position[1]]
            my_graph = {}
            my_graph[0] = 0
            node_counter = 1

    # print("\n\n\n\n")
    # print("my_graph\n: ")
    # print(my_graph)
    #
    # print("\n\n\n\n")
    # print("my_grid: \n")
    # print(my_grid)
    #
    # print("\n\n\n")
    # print("goal_node_number:")
    # print(goal_node_number)


    # goal_node_number = 712
    # my_graph = {0: {0: 0}, 1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 6: 5, 7: 4, 8: 6, 9: 8, 10: 8, 11: 10, 12: 11, 13: 12, 14: 13, 15: 14, 16: 15, 17: 9, 18: 16, 19: 18, 20: 19, 21: 19, 22: 16, 23: 22, 24: 21, 25: 24, 26: 7, 27: 1, 28: 25, 29: 28, 30: 29, 31: 30, 32: 31, 33: 32, 34: 33, 35: 33, 36: 35, 37: 36, 38: 12, 39: 17, 40: 34, 41: 40, 42: 37, 43: 41, 44: 43, 45: 42, 46: 23, 47: 38, 48: 44, 49: 48, 50: 49, 51: 49, 52: 51, 53: 50, 54: 53, 55: 46, 56: 53, 57: 53, 58: 52, 59: 53, 60: 39, 61: 58, 62: 61, 63: 53, 64: 53, 65: 53, 66: 45, 67: 20, 68: 55, 69: 9, 70: 54, 71: 66, 72: 53, 73: 62, 74: 62, 75: 29, 76: 46, 77: 67, 78: 0, 79: 71, 80: 53, 81: 68, 82: 73, 83: 26, 84: 54, 85: 79, 86: 71, 87: 81, 88: 53, 89: 74, 90: 85, 91: 89, 92: 82, 93: 87, 94: 93, 95: 43, 96: 53, 97: 53, 98: 87, 99: 98, 100: 53, 101: 33, 102: 89, 103: 92, 104: 53, 105: 66, 106: 69, 107: 94, 108: 86, 109: 84, 110: 90, 111: 26, 112: 53, 113: 92, 114: 76, 115: 103, 116: 95, 117: 109, 118: 113, 119: 115, 120: 53, 121: 118, 122: 119, 123: 99, 124: 122, 125: 107, 126: 53, 127: 124, 128: 53, 129: 122, 130: 91, 131: 127, 132: 17, 133: 117, 134: 108, 135: 66, 136: 53, 137: 131, 138: 47, 139: 90, 140: 125, 141: 2, 142: 70, 143: 133, 144: 53, 145: 137, 146: 131, 147: 95, 148: 140, 149: 148, 150: 121, 151: 145, 152: 53, 153: 139, 154: 110, 155: 153, 156: 45, 157: 143, 158: 53, 159: 149, 160: 53, 161: 28, 162: 157, 163: 151, 164: 162, 165: 148, 166: 164, 167: 102, 168: 53, 169: 166, 170: 91, 171: 162, 172: 159, 173: 48, 174: 163, 175: 47, 176: 53, 177: 163, 178: 141, 179: 17, 180: 169, 181: 99, 182: 21, 183: 172, 184: 53, 185: 133, 186: 155, 187: 102, 188: 150, 189: 121, 190: 134, 191: 180, 192: 53, 193: 75, 194: 67, 195: 138, 196: 183, 197: 177, 198: 159, 199: 147, 200: 53, 201: 166, 202: 181, 203: 89, 204: 186, 205: 197, 206: 205, 207: 196, 208: 53, 209: 187, 210: 191, 211: 196, 212: 174, 213: 27, 214: 188, 215: 209, 216: 53, 217: 210, 218: 173, 219: 54, 220: 172, 221: 130, 222: 31, 223: 204, 224: 53, 225: 26, 226: 214, 227: 221, 228: 133, 229: 210, 230: 206, 231: 157, 232: 53, 233: 209, 234: 204, 235: 212, 236: 167, 237: 217, 238: 211, 239: 79, 240: 53, 241: 15, 242: 230, 243: 173, 244: 229, 245: 242, 246: 207, 247: 209, 248: 53, 249: 199, 250: 123, 251: 68, 252: 251, 253: 185, 254: 9, 255: 222, 256: 53, 257: 235, 258: 69, 259: 252, 260: 202, 261: 3, 262: 146, 263: 223, 264: 53, 265: 157, 266: 226, 267: 260, 268: 246, 269: 230, 270: 15, 271: 247, 272: 53, 273: 260, 274: 195, 275: 191, 276: 201, 277: 135, 278: 269, 279: 146, 280: 53, 281: 7, 282: 268, 283: 113, 284: 263, 285: 99, 286: 231, 287: 170, 288: 53, 289: 244, 290: 183, 291: 69, 292: 197, 293: 223, 294: 165, 295: 289, 296: 53, 297: 19, 298: 250, 299: 227, 300: 271, 301: 226, 302: 9, 303: 284, 304: 53, 305: 206, 306: 257, 307: 99, 308: 286, 309: 282, 310: 125, 311: 301, 312: 53, 313: 111, 314: 39, 315: 24, 316: 311, 317: 166, 318: 237, 319: 234, 320: 53, 321: 45, 322: 294, 323: 17, 324: 300, 325: 212, 326: 318, 327: 177, 328: 53, 329: 276, 330: 92, 331: 268, 332: 275, 333: 319, 334: 195, 335: 333, 336: 53, 337: 311, 338: 332, 339: 324, 340: 266, 341: 324, 342: 339, 343: 252, 344: 53, 345: 282, 346: 195, 347: 190, 348: 347, 349: 269, 350: 263, 351: 1, 352: 53, 353: 6, 354: 46, 355: 255, 356: 292, 357: 148, 358: 222, 359: 154, 360: 53, 361: 161, 362: 267, 363: 345, 364: 303, 365: 337, 366: 335, 367: 365, 368: 53, 369: 315, 370: 338, 371: 369, 372: 114, 373: 20, 374: 220, 375: 257, 376: 53, 377: 299, 378: 53, 379: 82, 380: 363, 381: 101, 382: 217, 383: 341, 384: 53, 385: 364, 386: 309, 387: 114, 388: 194, 389: 383, 390: 348, 391: 283, 392: 53, 393: 389, 394: 348, 395: 274, 396: 293, 397: 212, 398: 397, 399: 17, 400: 53, 401: 377, 402: 110, 403: 261, 404: 338, 405: 244, 406: 147, 407: 47, 408: 53, 409: 27, 410: 157, 411: 194, 412: 371, 413: 407, 414: 359, 415: 60, 416: 53, 417: 246, 418: 359, 419: 25, 420: 349, 421: 370, 422: 335, 423: 290, 424: 53, 425: 397, 426: 406, 427: 349, 428: 166, 429: 338, 430: 381, 431: 405, 432: 53, 433: 305, 434: 22, 435: 404, 436: 434, 437: 300, 438: 53, 439: 12, 440: 53, 441: 127, 442: 346, 443: 318, 444: 251, 445: 405, 446: 347, 447: 137, 448: 53, 449: 279, 450: 42, 451: 445, 452: 333, 453: 449, 454: 353, 455: 366, 456: 53, 457: 313, 458: 222, 459: 334, 460: 276, 461: 24, 462: 322, 463: 47, 464: 53, 465: 417, 466: 442, 467: 154, 468: 244, 469: 372, 470: 382, 471: 86, 472: 53, 473: 466, 474: 325, 475: 394, 476: 221, 477: 326, 478: 15, 479: 414, 480: 53, 481: 186, 482: 350, 483: 188, 484: 187, 485: 366, 486: 151, 487: 364, 488: 53, 489: 342, 490: 485, 491: 186, 492: 61, 493: 322, 494: 117, 495: 290, 496: 53, 497: 483, 498: 380, 499: 60, 500: 148, 501: 407, 502: 260, 503: 361, 504: 53, 505: 81, 506: 109, 507: 451, 508: 443, 509: 124, 510: 412, 511: 249, 512: 53, 513: 462, 514: 507, 515: 475, 516: 473, 517: 110, 518: 235, 519: 318, 520: 53, 521: 514, 522: 388, 523: 94, 524: 485, 525: 474, 526: 266, 527: 471, 528: 53, 529: 524, 530: 60, 531: 22, 532: 455, 533: 491, 534: 115, 535: 529, 536: 53, 537: 420, 538: 20, 539: 215, 540: 316, 541: 523, 542: 522, 543: 365, 544: 53, 545: 515, 546: 498, 547: 53, 548: 69, 549: 76, 550: 218, 551: 394, 552: 53, 553: 508, 554: 535, 555: 426, 556: 545, 557: 554, 558: 206, 559: 489, 560: 53, 561: 261, 562: 356, 563: 253, 564: 332, 565: 340, 566: 521, 567: 37, 568: 53, 569: 423, 570: 476, 571: 445, 572: 290, 573: 453, 574: 521, 575: 390, 576: 53, 577: 487, 578: 298, 579: 86, 580: 83, 581: 557, 582: 393, 583: 372, 584: 53, 585: 404, 586: 222, 587: 381, 588: 483, 589: 401, 590: 110, 591: 508, 592: 53, 593: 303, 594: 471, 595: 238, 596: 164, 597: 542, 598: 589, 599: 574, 600: 53, 601: 53, 602: 434, 603: 386, 604: 556, 605: 298, 606: 573, 607: 396, 608: 53, 609: 395, 610: 498, 611: 582, 612: 572, 613: 47, 614: 285, 615: 603, 616: 53, 617: 306, 618: 515, 619: 565, 620: 255, 621: 305, 622: 116, 623: 379, 624: 53, 625: 474, 626: 329, 627: 566, 628: 371, 629: 286, 630: 537, 631: 202, 632: 53, 633: 174, 634: 545, 635: 535, 636: 236, 637: 189, 638: 635, 639: 521, 640: 53, 641: 274, 642: 334, 643: 398, 644: 595, 645: 465, 646: 595, 647: 606, 648: 53, 649: 42, 650: 316, 651: 55, 652: 102, 653: 141, 654: 281, 655: 157, 656: 53, 657: 213, 658: 631, 659: 611, 660: 375, 661: 597, 662: 492, 663: 13, 664: 53, 665: 227, 666: 630, 667: 154, 668: 543, 669: 295, 670: 234, 671: 385, 672: 53, 673: 319, 674: 645, 675: 55, 676: 380, 677: 598, 678: 129, 679: 666, 680: 53, 681: 355, 682: 581, 683: 679, 684: 559, 685: 513, 686: 598, 687: 365, 688: 682, 689: 429, 690: 643, 691: 190, 692: 682, 693: 618, 694: 276, 695: 325, 696: 688, 697: 4, 698: 581, 699: 659, 700: 41, 701: 267, 702: 374, 703: 565, 704: 696, 705: 148, 706: 250, 707: 665, 708: 194, 709: 131, 710: 17, 711: 689, 712: 704}
    #
    # my_grid = {0: [1, 1], 1: [2, 2], 2: [3, 3], 3: [4, 4], 4: [5, 5], 5: [6, 6], 6: [7, 7], 7: [4, 6], 8: [8, 8], 9: [7, 9], 10: [9, 9], 11: [10, 9], 12: [11, 8], 13: [12, 9], 14: [13, 10], 15: [14, 11], 16: [15, 12], 17: [6, 9], 18: [16, 11], 19: [17, 12], 20: [18, 11], 21: [18, 13], 22: [14, 13], 23: [13, 14], 24: [19, 14], 25: [20, 13], 26: [3, 7], 27: [3, 1], 28: [21, 13], 29: [22, 14], 30: [23, 15], 31: [24, 16], 32: [25, 17], 33: [26, 18], 34: [27, 17], 35: [25, 19], 36: [24, 20], 37: [23, 21], 38: [12, 7], 39: [5, 9], 40: [28, 18], 41: [29, 17], 42: [22, 22], 43: [30, 16], 44: [31, 17], 45: [23, 23], 46: [12, 15], 47: [13, 6], 48: [32, 18], 49: [33, 18], 50: [34, 19], 51: [34, 17], 52: [35, 16], 53: [35, 20], 54: [36, 19], 55: [11, 16], 56: [35, 19], 57: [36, 19], 58: [36, 15], 59: [36, 19], 60: [4, 9], 61: [37, 14], 62: [38, 13], 63: [36, 19], 64: [35, 19], 65: [35, 19], 66: [22, 24], 67: [19, 10], 68: [10, 16], 69: [8, 9], 70: [36, 18], 71: [21, 25], 72: [35, 19], 73: [39, 14], 74: [37, 12], 75: [21, 14], 76: [13, 16], 77: [18, 9], 78: [0, 1], 79: [22, 26], 80: [35, 19], 81: [9, 17], 82: [40, 15], 83: [2, 6], 84: [37, 18], 85: [23, 27], 86: [20, 26], 87: [8, 18], 88: [35, 19], 89: [38, 11], 90: [24, 27], 91: [39, 11], 92: [41, 14], 93: [9, 19], 94: [10, 20], 95: [29, 15], 96: [35, 19], 97: [36, 19], 98: [7, 17], 99: [6, 18], 100: [36, 19], 101: [27, 19], 102: [37, 10], 103: [42, 13], 104: [35, 19], 105: [22, 25], 106: [9, 9], 107: [9, 21], 108: [19, 27], 109: [38, 19], 110: [25, 26], 111: [2, 8], 112: [35, 19], 113: [42, 15], 114: [14, 17], 115: [43, 12], 116: [28, 15], 117: [39, 20], 118: [43, 16], 119: [44, 11], 120: [35, 19], 121: [44, 17], 122: [45, 10], 123: [5, 19], 124: [46, 9], 125: [8, 22], 126: [34, 19], 127: [47, 8], 128: [35, 19], 129: [45, 11], 130: [40, 10], 131: [48, 7], 132: [7, 9], 133: [40, 21], 134: [18, 28], 135: [22, 23], 136: [35, 19], 137: [49, 8], 138: [14, 5], 139: [23, 28], 140: [8, 23], 141: [2, 4], 142: [35, 18], 143: [41, 22], 144: [35, 19], 145: [50, 7], 146: [47, 6], 147: [29, 14], 148: [9, 24], 149: [8, 25], 150: [45, 18], 151: [51, 6], 152: [35, 19], 153: [23, 29], 154: [26, 25], 155: [24, 30], 156: [23, 24], 157: [42, 23], 158: [34, 19], 159: [7, 26], 160: [35, 19], 161: [22, 12], 162: [43, 24], 163: [52, 7], 164: [44, 25], 165: [10, 25], 166: [45, 26], 167: [36, 10], 168: [35, 19], 169: [46, 27], 170: [39, 12], 171: [42, 23], 172: [6, 27], 173: [31, 19], 174: [53, 6], 175: [12, 6], 176: [35, 19], 177: [53, 8], 178: [1, 4], 179: [5, 9], 180: [47, 28], 181: [5, 17], 182: [19, 13], 183: [5, 28], 184: [35, 19], 185: [40, 20], 186: [25, 31], 187: [37, 9], 188: [46, 19], 189: [45, 16], 190: [17, 29], 191: [48, 29], 192: [35, 19], 193: [21, 15], 194: [20, 9], 195: [15, 4], 196: [6, 29], 197: [54, 9], 198: [8, 26], 199: [30, 13], 200: [35, 19], 201: [46, 25], 202: [5, 16], 203: [37, 11], 204: [25, 32], 205: [55, 10], 206: [56, 11], 207: [5, 30], 208: [35, 19], 209: [36, 8], 210: [49, 30], 211: [7, 30], 212: [54, 5], 213: [4, 0], 214: [47, 20], 215: [35, 9], 216: [35, 19], 217: [48, 31], 218: [30, 20], 219: [35, 20], 220: [7, 28], 221: [41, 9], 222: [25, 15], 223: [24, 33], 224: [35, 19], 225: [3, 6], 226: [48, 21], 227: [41, 8], 228: [41, 22], 229: [50, 31], 230: [57, 12], 231: [43, 22], 232: [35, 19], 233: [37, 7], 234: [26, 33], 235: [55, 4], 236: [35, 11], 237: [47, 32], 238: [8, 31], 239: [23, 26], 240: [35, 19], 241: [14, 10], 242: [58, 11], 243: [31, 20], 244: [51, 32], 245: [57, 10], 246: [4, 31], 247: [35, 8], 248: [35, 19], 249: [31, 12], 250: [4, 19], 251: [9, 16], 252: [8, 16], 253: [41, 20], 254: [6, 9], 255: [25, 14], 256: [35, 19], 257: [56, 3], 258: [9, 9], 259: [7, 16], 260: [4, 16], 261: [5, 3], 262: [48, 5], 263: [23, 33], 264: [35, 19], 265: [41, 22], 266: [49, 22], 267: [3, 16], 268: [5, 32], 269: [58, 13], 270: [15, 11], 271: [34, 8], 272: [35, 19], 273: [4, 17], 274: [15, 5], 275: [49, 28], 276: [46, 26], 277: [22, 24], 278: [59, 13], 279: [46, 6], 280: [35, 19], 281: [4, 7], 282: [4, 33], 283: [42, 16], 284: [23, 34], 285: [5, 18], 286: [44, 21], 287: [39, 13], 288: [35, 19], 289: [50, 33], 290: [4, 27], 291: [9, 9], 292: [53, 10], 293: [25, 34], 294: [10, 26], 295: [49, 34], 296: [35, 19], 297: [16, 12], 298: [3, 20], 299: [41, 7], 300: [33, 7], 301: [49, 20], 302: [7, 8], 303: [22, 35], 304: [35, 19], 305: [55, 12], 306: [57, 4], 307: [5, 18], 308: [43, 20], 309: [3, 34], 310: [8, 21], 311: [50, 19], 312: [35, 19], 313: [1, 9], 314: [6, 9], 315: [19, 15], 316: [51, 18], 317: [46, 27], 318: [46, 33], 319: [27, 33], 320: [35, 19], 321: [24, 23], 322: [11, 26], 323: [5, 9], 324: [33, 6], 325: [53, 4], 326: [45, 32], 327: [52, 8], 328: [35, 19], 329: [47, 26], 330: [42, 14], 331: [6, 32], 332: [50, 27], 333: [28, 33], 334: [14, 3], 335: [29, 34], 336: [35, 19], 337: [51, 20], 338: [51, 26], 339: [34, 5], 340: [50, 22], 341: [32, 5], 342: [34, 4], 343: [9, 16], 344: [35, 19], 345: [5, 34], 346: [16, 3], 347: [16, 30], 348: [17, 31], 349: [58, 14], 350: [22, 32], 351: [2, 1], 352: [35, 19], 353: [8, 6], 354: [11, 14], 355: [26, 14], 356: [52, 11], 357: [9, 25], 358: [26, 15], 359: [27, 24], 360: [35, 19], 361: [23, 12], 362: [2, 16], 363: [5, 35], 364: [21, 36], 365: [52, 19], 366: [30, 33], 367: [51, 19], 368: [35, 19], 369: [19, 16], 370: [52, 25], 371: [19, 17], 372: [15, 18], 373: [18, 10], 374: [8, 28], 375: [56, 2], 376: [35, 19], 377: [40, 6], 378: [36, 19], 379: [39, 15], 380: [6, 35], 381: [26, 20], 382: [47, 31], 383: [31, 4], 384: [35, 19], 385: [20, 37], 386: [2, 35], 387: [14, 18], 388: [21, 8], 389: [31, 3], 390: [18, 31], 391: [41, 17], 392: [35, 19], 393: [30, 2], 394: [16, 31], 395: [16, 5], 396: [25, 35], 397: [55, 6], 398: [55, 7], 399: [7, 9], 400: [35, 19], 401: [39, 5], 402: [24, 27], 403: [4, 3], 404: [50, 25], 405: [52, 33], 406: [30, 14], 407: [14, 7], 408: [35, 19], 409: [4, 1], 410: [43, 24], 411: [19, 8], 412: [20, 18], 413: [14, 8], 414: [28, 23], 415: [3, 9], 416: [35, 19], 417: [3, 31], 418: [26, 25], 419: [20, 12], 420: [59, 15], 421: [53, 25], 422: [28, 34], 423: [4, 26], 424: [35, 19], 425: [56, 6], 426: [31, 15], 427: [59, 14], 428: [45, 25], 429: [52, 27], 430: [26, 21], 431: [53, 32], 432: [35, 19], 433: [54, 11], 434: [15, 14], 435: [50, 26], 436: [15, 15], 437: [34, 7], 438: [34, 19], 439: [10, 8], 440: [35, 19], 441: [47, 9], 442: [17, 3], 443: [45, 34], 444: [9, 17], 445: [53, 34], 446: [15, 30], 447: [50, 8], 448: [35, 19], 449: [45, 5], 450: [21, 22], 451: [54, 35], 452: [29, 32], 453: [45, 4], 454: [8, 7], 455: [31, 32], 456: [35, 19], 457: [2, 9], 458: [26, 15], 459: [14, 4], 460: [47, 26], 461: [20, 14], 462: [12, 25], 463: [13, 7], 464: [35, 19], 465: [2, 30], 466: [18, 2], 467: [25, 24], 468: [51, 33], 469: [15, 17], 470: [46, 31], 471: [19, 25], 472: [35, 19], 473: [19, 2], 474: [53, 3], 475: [15, 32], 476: [41, 10], 477: [44, 32], 478: [14, 10], 479: [28, 22], 480: [35, 19], 481: [25, 30], 482: [21, 31], 483: [46, 18], 484: [37, 8], 485: [31, 34], 486: [50, 5], 487: [21, 35], 488: [35, 19], 489: [35, 3], 490: [30, 35], 491: [26, 31], 492: [37, 13], 493: [12, 27], 494: [40, 21], 495: [3, 28], 496: [35, 19], 497: [47, 18], 498: [7, 36], 499: [3, 9], 500: [9, 25], 501: [15, 7], 502: [3, 16], 503: [24, 11], 504: [35, 19], 505: [10, 18], 506: [39, 19], 507: [54, 36], 508: [44, 35], 509: [46, 8], 510: [21, 17], 511: [30, 11], 512: [35, 19], 513: [11, 24], 514: [55, 36], 515: [14, 33], 516: [19, 1], 517: [26, 25], 518: [55, 3], 519: [47, 34], 520: [35, 19], 521: [56, 37], 522: [22, 7], 523: [11, 20], 524: [32, 33], 525: [54, 3], 526: [49, 23], 527: [19, 24], 528: [35, 19], 529: [33, 33], 530: [3, 9], 531: [13, 13], 532: [31, 31], 533: [26, 32], 534: [43, 11], 535: [34, 33], 536: [35, 19], 537: [58, 16], 538: [18, 10], 539: [34, 9], 540: [51, 19], 541: [11, 21], 542: [22, 6], 543: [53, 20], 544: [35, 19], 545: [14, 34], 546: [6, 37], 547: [36, 19], 548: [9, 9], 549: [14, 16], 550: [29, 20], 551: [16, 30], 552: [35, 19], 553: [45, 36], 554: [35, 32], 555: [32, 14], 556: [13, 35], 557: [34, 31], 558: [56, 12], 559: [36, 2], 560: [35, 19], 561: [6, 2], 562: [51, 12], 563: [42, 19], 564: [50, 28], 565: [51, 23], 566: [55, 38], 567: [23, 20], 568: [35, 19], 569: [3, 25], 570: [42, 10], 571: [53, 35], 572: [5, 27], 573: [44, 3], 574: [57, 37], 575: [19, 32], 576: [35, 19], 577: [20, 34], 578: [3, 21], 579: [21, 27], 580: [2, 7], 581: [35, 30], 582: [29, 2], 583: [15, 19], 584: [35, 19], 585: [49, 25], 586: [25, 16], 587: [26, 19], 588: [46, 17], 589: [39, 4], 590: [24, 27], 591: [43, 36], 592: [35, 19], 593: [23, 36], 594: [19, 26], 595: [8, 32], 596: [45, 26], 597: [23, 5], 598: [40, 3], 599: [58, 38], 600: [35, 19], 601: [34, 19], 602: [16, 14], 603: [2, 36], 604: [14, 36], 605: [2, 20], 606: [44, 2], 607: [26, 36], 608: [35, 19], 609: [17, 6], 610: [8, 37], 611: [28, 3], 612: [5, 26], 613: [13, 7], 614: [6, 18], 615: [1, 37], 616: [35, 19], 617: [57, 5], 618: [13, 32], 619: [52, 23], 620: [25, 15], 621: [54, 13], 622: [29, 15], 623: [39, 16], 624: [35, 19], 625: [52, 2], 626: [47, 25], 627: [54, 39], 628: [19, 16], 629: [43, 21], 630: [57, 17], 631: [6, 16], 632: [35, 19], 633: [53, 5], 634: [15, 34], 635: [34, 34], 636: [34, 11], 637: [44, 16], 638: [33, 35], 639: [56, 36], 640: [35, 19], 641: [15, 4], 642: [13, 2], 643: [56, 7], 644: [7, 32], 645: [1, 30], 646: [9, 33], 647: [43, 2], 648: [35, 19], 649: [22, 21], 650: [51, 17], 651: [10, 16], 652: [38, 10], 653: [2, 3], 654: [5, 7], 655: [41, 22], 656: [35, 19], 657: [3, 0], 658: [5, 16], 659: [27, 4], 660: [57, 1], 661: [24, 5], 662: [36, 13], 663: [11, 9], 664: [35, 19], 665: [42, 8], 666: [58, 18], 667: [25, 26], 668: [53, 19], 669: [48, 35], 670: [25, 33], 671: [20, 38], 672: [35, 19], 673: [28, 33], 674: [1, 31], 675: [12, 16], 676: [6, 34], 677: [39, 2], 678: [46, 12], 679: [57, 19], 680: [35, 19], 681: [26, 13], 682: [36, 29], 683: [57, 20], 684: [36, 3], 685: [10, 23], 686: [40, 2], 687: [52, 18], 688: [35, 28], 689: [53, 28], 690: [57, 8], 691: [16, 28], 692: [37, 29], 693: [12, 31], 694: [46, 27], 695: [53, 3], 696: [35, 27], 697: [6, 5], 698: [35, 31], 699: [27, 5], 700: [28, 17], 701: [2, 16], 702: [9, 29], 703: [50, 23], 704: [35, 26], 705: [10, 24], 706: [4, 18], 707: [42, 9], 708: [21, 10], 709: [48, 6], 710: [6, 8], 711: [54, 27], 712: [35, 25]}





    ##---Create static bodies to mark the start and the goal position---##
    body = my_world.CreateStaticBody(position=robot_start_position)
    box = body.CreatePolygonFixture(vertices=vertices2, density=1)
    box.sensor = True
    body.userData = {'color': 'start_position'}

    body2 = my_world.CreateStaticBody(position=goal_position)
    box2 = body2.CreatePolygonFixture(vertices=vertices2, density=1)
    box2.sensor = True
    body2.userData = {'color': 'goal_position'}

    ##---Create static bodies to mark the nodes in the tree with small "dots"---##
    ##---(it does not collide with static or dynamic bodies)---##
    for key, value in my_grid.items():
        x = value[0]
        y = value[1]
        body3 = my_world.CreateStaticBody(position=(x,y))
        box3 = body3.CreatePolygonFixture(box=(0.098, 0.098), density=1)
        box3.sensor = True
        body3.userData = {'color': 'node_marker'}



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


    # # Draw small circles along the shortest path
    for number in shortest_path:
        x = my_grid[number][0]
        y = my_grid[number][1]
        #print (x, y)
        body = my_world.CreateKinematicBody(position=(x,y))
        box = body.CreatePolygonFixture(box=(0.098, 0.098), density=1, friction=0.3)
        box.sensor = True
        body.userData = {'color': 'shortest_path'}


    ##---Create a dynamic body for the robot---##
    dynamic_body = my_world.CreateDynamicBody(position=robot_start_position, angle=0)
    box = dynamic_body.CreatePolygonFixture(vertices=vertices2, density=1, friction=0.3)
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
    print("number of nodes in tree: %d" % (node_counter) )


##---main game loop---##
my_world = World()
grid = my_world.my_grid

running = True
goal_reached = False
counter = 0
target_coordinate_list = []

# Get shortest path
targetlist = my_world.shortest_path

##---Create list of the shortest path cells: for each cell get the x coordinate, y coordinate and cell number---##
for n in range(len(targetlist)):
    list = grid[targetlist[n]]
    list.append(targetlist[n])
    target_coordinate_list.append(list)
print("targets:::::::::::")
print(target_coordinate_list)
print("Length of targetlist: ", len(targetlist))
goal_node = len(targetlist)-1


##print(grid)
while running:
    # Check the event queue
    for event in pygame.event.get():
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            # The user closed the window or pressed escape
            running = False

    my_world.screen.fill((0, 0, 0, 0))
    robot = my_world.my_world.bodies[-1]

    # Draw the world
    for body in (my_world.my_world.bodies):  # or: world.bodies
        # The body gives us the position and angle of its shapes
        for fixture in body.fixtures:
            shape = fixture.shape
            vertices = [(body.transform * v) * my_world.PPM for v in shape.vertices]
            vertices = [(v[0], my_world.SCREEN_HEIGHT - v[1]) for v in vertices]

            #pygame.draw.polygon(my_world.screen, my_world.colors[body.type], vertices)
            pygame.draw.polygon(my_world.screen, my_world.colors[body.userData['color']], vertices)
    my_world.my_world.Step(my_world.TIME_STEP, 10, 10)
    # print(len(my_world.my_world.bodies))

    pygame.display.flip()
    my_world.clock.tick(my_world.TARGET_FPS)

    ##---The target is the actual element of the target coordinate list---##
    target = target_coordinate_list[counter]

    if (goal_reached == False):
        ##---Calculate x, y distances of the actual target and the robot body and set velocity to move towards the target---##
        if (np.square(target[0] - robot.worldCenter.x) > 0.01 or np.square(target[1] - robot.worldCenter.y) > 0.01):
            direction_x = target[0] - robot.worldCenter.x
            direction_y = target[1] - robot.worldCenter.y
            distance = np.sqrt(np.square(direction_x)+np.square(direction_y))
            speed = 1.9 * 1/distance

            my_world.dynamic_body.linearVelocity = (speed*direction_x, speed*direction_y)
            my_world.my_world.Step(my_world.TIME_STEP, 10, 10)
        ##---If robot reaches the actual target point---##
        else:
            counter+=1
        ##---If robot reaches the final goal node (within a certain threshold)---##
        if (np.square(target_coordinate_list[goal_node][0] - robot.worldCenter.x) < 0.01 and np.square(target_coordinate_list[goal_node][1] - robot.worldCenter.y) < 0.01):
            goal_reached=True

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
