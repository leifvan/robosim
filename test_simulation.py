import unittest
from simulation import Simulation, BoundarySegment, MPLSimulationRenderer
from random import random, gauss
from math import pi
import numpy as np


def gauss_random_angle_strategy(angle, dt):
    return (angle + pi + gauss(0, 0.5)*pi/2) % (2*pi)


def totally_random_angle_strategy(angle, dt):
    return 2*pi*random()


class TestRectangleSimulation(unittest.TestCase):
    def setUp(self):
        bounds = BoundarySegment.closed_chain([[-1,1],[1,1],[1,-1],[-1,-1]])
        self.sim = Simulation(bounds, totally_random_angle_strategy)
        #self.sim.renderer = MPLSimulationRenderer()

    def test_rectangle_simulation(self):
        print('S', self.sim.robot.pos, self.sim.robot.angle)

        for i in range(2000):
            old_pos = np.copy(self.sim.robot.pos)
            self.sim.do_step()
            print(i, self.sim.robot.pos, self.sim.robot.angle)
            self.assertLessEqual(np.linalg.norm(self.sim.robot.pos, ord=np.inf), 1, msg='robot outside bounds')

            #self.assertGreater(, 1e-5, msg='robot got stuck')


class TestPolygonSimulation(unittest.TestCase):
    def setUp(self):
        bounds = BoundarySegment.closed_chain([[-6,2],[-4,6],[0,5],[5,7],[10,1],[5,-4],[4,0],[0,-4],[-3,-3],[-9,-5],
                                               [-10,-1],[-6,-1]])
        self.sim = Simulation(bounds, totally_random_angle_strategy)
        #self.sim.renderer = MPLSimulationRenderer()

    def test_polygon_simulation(self):
        print('S', self.sim.robot.pos, self.sim.robot.angle)
        for i in range(2000):
            old_pos = np.copy(self.sim.robot.pos)
            self.sim.do_step()
            print(i, self.sim.robot.pos, self.sim.robot.angle)
            self.assertLessEqual(np.linalg.norm(self.sim.robot.pos), 23.32, msg='robot outside bounds')
            #self.assertGreater(np.linalg.norm(self.sim.robot.pos - old_pos), 1e-5, msg='robot got stuck')


if __name__ == '__main__':
    unittest.main()
