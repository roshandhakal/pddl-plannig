"""
This file is to create primitives for
the environment. The codes are similar
to that of in example of continous_tamp
within PDDLStream. The modification here
is we created top down view of the environment
so, there are few changes. Furthermore, we also
added obstacles in the environment hence we have
also used motion planning algorithm.

For more info on PDDLStream
Continous TAMP primitives:
https://github.com/caelan/pddlstream/blob/main/examples/continuous_tamp/primitives.py
For more info on Motion Planners:
https://github.com/caelan/motion-planners

"""

from collections import namedtuple
import numpy as np
from motion_planners.lazy_prm import lazy_prm
import sys
from examples.continuous_tamp.primitives import TAMPState, forward_kin

LIVING_ROOM = "living room"
BLOCK_WIDTH = 40
BLOCK_LENGTH = BLOCK_WIDTH
#GROUND_Y = 0.0

SUCTION_WIDTH = 80
GRASP = -np.array([0, 0])
#CARRY_Y = 20
APPROACH = -np.array([0, 0])

MOVE_COST = 1.0
COST_PER_DIST = 1.0
DISTANCE_PER_TIME = 100.0


def get_block_box(b, p=np.zeros(2)):
    extent = np.array([(BLOCK_WIDTH, BLOCK_LENGTH)])
    lower = p - extent / 2
    upper = p + extent / 2.0
    lower[lower < 0] = 0
    lower[lower > 500] = 500
    upper[upper < 0] = 0
    upper[upper > 500] = 500

    return lower, upper


def get_robot_interval(p=np.zeros(2)):
    extent = np.array([(70, SUCTION_WIDTH)])
    lower = p - extent / 2
    upper = p + extent / 2.0
    lower[lower < 0] = 0
    lower[lower > 500] = 500
    upper[upper < 0] = 0
    upper[upper > 500] = 500
    return lower[0], upper[0]


def interval_contains(i1, i2):
    """
    :param i1: The container interval
    :param i2: The possibly contained interval
    :return:
    """
    val = (
        i1[0][0] <= i2[0][0]
        and i2[1][0] <= i1[1][0]
        and i1[0][1] <= i2[0][1]
        and i2[1][1] <= i1[1][1]
    )
    return val


def interval_overlap(i1, i2):
    val = (
        i1[0][0] <= i2[1][0]
        and i1[0][1] <= i2[1][1]
        and i1[1][0] >= i2[0][0]
        and i1[1][1] >= i2[0][1]
    )
    return val


def get_block_interval(b, p):
    l1, u1 = get_block_box(b, p)
    return l1[0], u1[0]

##################################################
def distance_fn(q1, q2):
    ord = 1  # 1 | 2
    return MOVE_COST + COST_PER_DIST * np.linalg.norm(q2 - q1, ord=ord)

def collision_test(b1, p1, b2, p2):
    if b1 == b2:
        return False
    return interval_overlap(get_block_interval(b1, p1), get_block_interval(b2, p2))


def get_region_test(regions):
    def test(b, p, r):
        return interval_contains(regions[r], get_block_interval(b, p))

    return test


def sample_region(b, region):
    lower, upper = np.array(region, dtype=float)
    x1, y1 = lower[0], lower[1]
    x2, y2 = upper[0], upper[1]
    x = np.random.uniform(x1 + 20, x2 - 20)
    y = np.random.uniform(y1 + 20, y2 - 20)

    return np.array([x, y])


def get_pose_gen(regions):
    def gen_fn(b, r):
        while True:
            p = sample_region(b, regions[r])
            yield (p,)

    return gen_fn


def contains(robot_box, box):
    if (
        (robot_box[1][0] <= box[0][0])
        or (robot_box[1][1] <= box[0][1])
        or (robot_box[0][0] >= box[1][0])
        or (robot_box[0][1] >= box[1][1])
    ):
        return False

    return True


def robot_collides(point, boxes):
    if len(boxes) == 0:
        return False
    robot_box = get_robot_interval(point)
    return any(contains(robot_box, box) for box in boxes)


def get_delta(a, b):
    return np.array(b) - np.array(a)


def sample_line(segment, step_size=0.2):
    (a, b) = segment
    diff = get_delta(a, b)
    dist = np.linalg.norm(diff)
    for i in np.arange(0.0, dist, step_size):
        yield tuple(np.array(a) + i * diff / dist)
    yield b


# ==============================================================================
roadmap = []


def get_extend_fn(obstacles):
    collision_fn = get_collision_fn(obstacles)

    def extend_fn(a, b):
        path = [a]
        for q in sample_line(segment=(a, b)):
            yield q
            if collision_fn(q):
                path = None
            if path is not None:
                roadmap.append((path[-1], q))
                path.append(q)

    return extend_fn


def sample_box():
    lower = np.array([30, 30])
    upper = np.array([470, 470])
    return np.random.random(2) * (upper - lower) + lower


def get_sample_fn(obstacles, **kwargs):
    collision_fn = get_collision_fn(obstacles)

    def region_gen():
        while True:
            q = np.array(sample_box())
            if collision_fn(q):
                continue
            return q

    return region_gen


def get_collision_fn(obstacles):
    def collision_fn(q):
        if robot_collides(q, obstacles):
            return True
        return False

    return collision_fn


def plan_ballmotion(b, q1, p2):
    motion = [q1, p2]
    return (motion, )


def plan_motion(q1, q2, obstacles):
    if len(obstacles) == 0:
        t = [q1, q2]
        return (t,)
    obstacle_dims = list(obstacles.values())
    obstacles = []
    for obs in obstacle_dims:
        if type(obs[0]) is int:
            fixed_width = 60
            x, y = obs[0], obs[1]
            x1, x2, y1, y2 = (
                x - fixed_width / 2,
                x + fixed_width / 2,
                y - fixed_width / 2,
                y + fixed_width / 2,
            )
            obstacle = [(x1, y1), (x2, y2)]
            obstacles.append(obstacle)
        else: 
            obstacles.extend(obs)
    extend_fn = get_extend_fn(obstacles)
    sample_fn = get_sample_fn(obstacles)
    collision_fn = get_collision_fn(obstacles)
    path = lazy_prm(q1, q2, sample_fn, extend_fn, collision_fn, num_samples=10000)
    return (path[0],)


##################################################
TAMPProblem = namedtuple(
    "TAMPProblem", ["initial", "regions", "obstacles", "goal_conf", "goal_regions"]
)

GOAL_NAME = 'touchdown'
TABLE_NAME = 'table'

INITIAL_CONF = np.array([350, 450])
INITIAL_CONF2 = np.array([200, 600])
GOAL_CONF = INITIAL_CONF
REGIONS = {
'field': [(10, 10), (450, 700)],
GOAL_NAME: [(10, 10), (450, 100)],
"teammate": [(330, 430), (370, 470)],
}

#OBSTACLES = []
OBSTACLES = {
    'opp2': [200, 350],
    'opp3': [300, 350],
    'opp4': [100, 350],
    'opp5': [300, 250],
    'opp6': [150, 250],
    }

envs = list(REGIONS.keys())
envs.remove(GOAL_NAME)
ENVIRONMENT_NAMES = envs

def tight():
    regions = {
            "Field" : [(10, 10), (450, 700)],
            "Defense Zone": [(10, 500), (450, 700)],
            "teammate": [(330, 430), (370, 470)],
            "TouchDown": [(10, 10), (450, 100)]
            }
    obstacles = []
    robots = ["qb", "wr"]
    confs = [INITIAL_CONF2, INITIAL_CONF]
    initial_confs = dict(zip(robots, confs))
    poses = [(200, 595)]
    objects = ['ball1', 'ball2']
    initial = TAMPState(initial_confs, {}, dict(zip(objects, poses)))
    goal_regions = {objects[0]: "TouchDown"} 

    return TAMPProblem(initial, regions, obstacles, GOAL_CONF, goal_regions)


def run_ball():
    regions = {
            "Field" : [(10, 10), (450, 700)],
            "Defense Zone": [(10, 500), (450, 700)],
            "TouchDown": [(10, 10), (450, 100)]
            }
    obstacles = {
    'opp2': [200, 350],
    'opp3': [300, 350],
    'opp4': [100, 350],
    'opp5': [300, 250],
    'opp6': [150, 250],
    }
    confs = [INITIAL_CONF, INITIAL_CONF2]
    goals = ["teammate", "TouchDown"]
    robots = ["ghost", "qb", "wr"]
    confs = [INITIAL_CONF2, INITIAL_CONF2, INITIAL_CONF]
    robots = ["wr", "qb"]
    initial_confs = dict(zip(robots, confs))
    poses = [(350, 445)]
    objects = ['ball']
    initial = TAMPState(initial_confs, {}, dict(zip(objects, poses)))
    goal_regions = {objects[0]: "TouchDown"} 

    return TAMPProblem(initial, regions, obstacles, GOAL_CONF, goal_regions)

def tight1():
    prob1 = pass_ball()
    prob2 = run_ball()

    print(prob1 + prob2)
    return prob1 + prob2

PROBLEMS = [tight]