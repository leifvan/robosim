import numpy as np
from math import atan2, cos, sin, pi
from operator import itemgetter
import matplotlib.pyplot as plt
import matplotlib.lines as mlines


class Simulation:

    def __init__(self, bounds, new_angle_strategy):
        self.boundary = [BoundarySegment(b1,b2) for b1,b2 in bounds]
        self.robot = Robot(new_angle_strategy)
        self.time = 0
        self.renderer = SimulationRenderer()

    @property
    def bounding_box(self):
        # TODO this is horribly inefficient, improve
        sx = min(s.start_point[0] for s in self.boundary)
        ex = min(s.end_point[0] for s in self.boundary)
        sy = min(s.start_point[1] for s in self.boundary)
        ey = min(s.end_point[1] for s in self.boundary)
        min_x = min(sx,ex)
        min_y = min(sy,ey)

        sx = max(s.start_point[0] for s in self.boundary)
        ex = max(s.end_point[0] for s in self.boundary)
        sy = max(s.start_point[1] for s in self.boundary)
        ey = max(s.end_point[1] for s in self.boundary)
        max_x = max(sx, ex)
        max_y = max(sy, ey)

        return np.array([min_x, max_x]), np.array([min_y, max_y])

    def do_step(self):
        # find next collision by projecting robot position onto boundary segments
        point_distance_pairs = [(i, bseg.project_point(self.robot.pos, self.robot.velocity))
                                for i, bseg in enumerate(self.boundary)]
        point_distance_pairs = [(i, *pdp) for i, pdp in point_distance_pairs if pdp is not None]

        # TODO add case if no collision is found
        seg_idx, collision_point, collision_dist = min(point_distance_pairs, key=itemgetter(2))

        print('hit segment', seg_idx, collision_point, collision_dist)
        if collision_dist < 0.1:
            print("don't move")
            dt = 0
        else:
            self.robot.pos += self.robot.velocity * collision_dist * 0.99 - 0.1 * self.boundary[seg_idx]._normal
            print('new readings',self.boundary[seg_idx].project_point(self.robot.pos, self.robot.velocity))
            dt = collision_dist
            self.time += dt

        self.robot.handle_collision(dt)
        self.renderer.render(self)


class SimulationRenderer:
    def render(self, sim):
        pass


class MPLSimulationRenderer:
    def __init__(self):
        self.last_robot_pos = np.array([0,0])
        self.lines = []

    def render(self, sim):
        self.lines.append([[self.last_robot_pos[0], sim.robot.pos[0]], [self.last_robot_pos[1], sim.robot.pos[1]]])
        self.last_robot_pos = np.copy(sim.robot.pos)

        fig, ax = plt.subplots(1, 1)
        bbox = sim.bounding_box
        ax.set_xlim(bbox[0]*1.1)
        ax.set_ylim(bbox[1]*1.1)

        for bseg in sim.boundary:
            l = mlines.Line2D([bseg.start_point[0], bseg.end_point[0]],[bseg.start_point[1], bseg.end_point[1]])
            l.set_color('red')
            ax.add_line(l)

            # midpoint
            mp = 0.5*(bseg.start_point + bseg.end_point)
            #mp_n = mp + bseg._normal*0.5
            #n = mlines.Line2D([mp[0], mp_n[0]],[mp[1],mp_n[1]])
            #n.set_color('yellow')
            plt.arrow(*mp, *bseg._normal, color='yellow')

        for i,l in enumerate(self.lines[-10:]):
            l = mlines.Line2D(l[0], l[1])
            l.set_color((1-i/min(len(self.lines),10))*np.ones(3))
            ax.add_line(l)
        plt.show()


class Robot:
    def __init__(self, new_angle_strategy):
        self.pos = np.array([0,0], dtype=np.float)
        self.angle = 0
        self.speed = 1
        self.new_angle_strategy = new_angle_strategy

    @property
    def velocity(self):
        return np.array([cos(self.angle), sin(self.angle)], dtype=np.float)

    def handle_collision(self, dt):
        self.angle = self.new_angle_strategy(self.angle, dt)


class BoundarySegment:

    def __init__(self, sp, ep):
        self.start_point = np.array(sp, dtype=np.float)
        self.end_point = np.array(ep, dtype=np.float)

        print(self.end_point[1] - self.start_point[1], self.end_point[0] - self.start_point[0])
        self._angle = atan2(self.end_point[1] - self.start_point[1], self.end_point[0] - self.start_point[0]) + pi/2
        self._angle = self._angle % (2*pi)
        self._normal = np.array([cos(self._angle), sin(self._angle)], dtype=np.float)
        self._dist = np.dot(self.start_point, self._normal)

        assert np.allclose(np.dot(self._normal, self.start_point) - self._dist, 0)
        assert np.allclose(np.dot(self._normal, self.end_point) - self._dist, 0)

    def __getitem__(self, item):
        if item == 0:
            return self.start_point
        elif item == 1:
            return self.end_point
        raise IndexError('Index must be integer 0 or 1.')

    @classmethod
    def closed_chain(cls, points):
        return [BoundarySegment(p1,p2) for p1,p2 in zip(points, points[1:])] + [BoundarySegment(points[-1], points[0])]

    def project_point(self, point, direction):
        """

        :param point:
        :param direction:
        :return: Tuple (projected_point, signed_distance_per_direction_length)
        """
        pr = point + (self._dist - np.dot(point, self._normal)) / np.dot(direction, self._normal) * direction
        dist = np.linalg.norm(pr - point) / np.linalg.norm(direction)

        # check if projection is between start and end point
        x_left, x_right = min(self.start_point[0], self.end_point[0]), max(self.start_point[0], self.end_point[0])
        y_left, y_right = min(self.start_point[1], self.end_point[1]), max(self.start_point[1], self.end_point[1])
        is_vertical_segment = np.allclose(self._normal,[1,0]) or np.allclose(self._normal,[-1,0])
        #midpoint = 0.5*(self.start_point + self.end_point)
        pr_angle = atan2(pr[1]-point[1], pr[0]-point[0])
        direction_angle  = atan2(direction[1], direction[0])
        is_in_front_of = -pi/2 < (pr_angle - direction_angle) < pi/2

        if (not is_vertical_segment and x_left < pr[0] < x_right
             or is_vertical_segment and y_left < pr[1] < y_right) and is_in_front_of:
            return pr, dist
        return None
