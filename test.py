import numpy as np
import matplotlib.pyplot as plt
from random import random
from math import pi, cos, sin

MAP_SIZE = 100

obstacle_map = np.zeros((MAP_SIZE, MAP_SIZE), np.bool)

obstacle_map[40:60, 0:40] = 1
obstacle_map[60:80, 20:40] = 1
# obstacle_map[40:180, 240:260] = 1

ROBOT_SIZE = 2
ROBOT_SPEED = 1

RENDER_EVERY = 20
DIRT_ROBUSTNESS = 20

fig, axs = plt.subplots(1,1)
map_plot = axs.imshow(np.ones((MAP_SIZE, MAP_SIZE)), vmin=0, vmax=DIRT_ROBUSTNESS)
plt.show(block=False)

# consider uncertainties in motor performance
# consider robot bigger than cleaned surface
# improve simulation

# http://www2.informatik.uni-freiburg.de/~hess/publications/hess14icra/hess14icra.pdf


def robot_simulation(fixed_angle, random_angle_scale, render=False):
    cleanliness = 0

    dirt_map = np.ones((MAP_SIZE, MAP_SIZE)) * DIRT_ROBUSTNESS
    dirt_map[obstacle_map] = 0

    i = 0

    robot_angle = 2 * pi * random()
    robot_pos = [MAP_SIZE // 2, MAP_SIZE // 2]

    while cleanliness < 0.9:
        for _ in range(RENDER_EVERY):
            robot_pos[0] += cos(robot_angle) * ROBOT_SPEED
            robot_pos[1] += sin(robot_angle) * ROBOT_SPEED

            coll = False

            if robot_pos[0] < ROBOT_SIZE:
                robot_pos[0] = ROBOT_SIZE
                coll = True
            elif robot_pos[0] > MAP_SIZE - ROBOT_SIZE:
                robot_pos[0] = MAP_SIZE - ROBOT_SIZE
                coll = True

            if robot_pos[1] < ROBOT_SIZE:
                robot_pos[1] = ROBOT_SIZE
                coll = True
            elif robot_pos[1] > MAP_SIZE - ROBOT_SIZE:
                robot_pos[1] = MAP_SIZE - ROBOT_SIZE
                coll = True

            xx, yy = np.mgrid[int(robot_pos[0])-ROBOT_SIZE:int(robot_pos[0])+ROBOT_SIZE,
                              int(robot_pos[1])-ROBOT_SIZE:int(robot_pos[1])+ROBOT_SIZE]

            circle = (xx - robot_pos[0]) ** 2 + (yy - robot_pos[1]) ** 2
            mask = circle <= ROBOT_SIZE ** 2

            local_obstacle_map = obstacle_map[int(robot_pos[0])-ROBOT_SIZE:int(robot_pos[0])+ROBOT_SIZE,
                                              int(robot_pos[1])-ROBOT_SIZE:int(robot_pos[1])+ROBOT_SIZE]

            if np.max(local_obstacle_map[mask]) == 1:
                coll = True

            if coll:
                robot_angle += (fixed_angle + random_angle_scale*(2*random()-1)) % (2*pi)

            local_dirt_map = dirt_map[int(robot_pos[0])-ROBOT_SIZE:int(robot_pos[0])+ROBOT_SIZE,
                                      int(robot_pos[1])-ROBOT_SIZE:int(robot_pos[1])+ROBOT_SIZE]
            local_dirt_map[mask] -= 1
            local_dirt_map[local_dirt_map < 0] = 0

            if i % 5000 == 0:
                cleanliness = 1- np.mean(dirt_map) / DIRT_ROBUSTNESS
                #print(f'step {i} cleanliness {cleanliness:.2%}')

            i += 1

        if render:
            map_plot.set_data(dirt_map)
            fig.canvas.draw()

    cleanliness = 1 - np.mean(dirt_map) / DIRT_ROBUSTNESS
    print(f'final cleanliness: {cleanliness:.2%} at step {i}')
    return i


vals = [robot_simulation(fixed_angle=0, random_angle_scale=pi, render=False) for _ in range(20)]
m = sum(vals)/len(vals)

print(f'mean steps for random strategy: {m:.2f}')
print('-'*20)


vals = [robot_simulation(fixed_angle=pi, random_angle_scale=pi/2, render=False) for _ in range(20)]
m = sum(vals)/len(vals)
print(f'mean steps for half-plane strategy: {m:.2f}')
print('-'*20)


vals = [robot_simulation(fixed_angle=pi/2, random_angle_scale=pi/4, render=False) for _ in range(20)]
m = sum(vals)/len(vals)
print(f'mean steps for 90° strategy: {m:.2f}')
print('-'*20)
