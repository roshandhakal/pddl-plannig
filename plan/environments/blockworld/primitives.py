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

from examples.continuous_tamp.primitives import make_blocks, TAMPState

GROUND_NAME = "grey"
BLOCK_WIDTH = 40
BLOCK_LENGTH = BLOCK_WIDTH
GROUND_Y = 0.0

SUCTION_WIDTH = 60
GRASP = -np.array([0, 0])
CARRY_Y = 150
APPROACH = -np.array([5, 5])

MOVE_COST = 10.0
COST_PER_DIST = 1/3
DISTANCE_PER_TIME = 100


def get_block_box(b, p=np.zeros(2)):
    extent = np.array([(BLOCK_WIDTH, BLOCK_LENGTH)])
    lower = p - extent / 2
    upper = p + extent / 2.0

    return lower, upper


def get_robot_interval(p=np.zeros(2)):
    extent = np.array([(SUCTION_WIDTH, SUCTION_WIDTH)])
    lower = p - extent / 2
    upper = p + extent / 2.0
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
    if (
        i1[1][0] <= i2[0][0]
        or i1[1][1] <= i2[0][1]
        or i1[0][0] >= i2[1][0]
        or i1[0][1] >= i2[1][1]
    ):
        return False
        
    return True 



def get_block_interval(b, p):
    l1, u1 = get_block_box(b, p)
    return l1[0], u1[0]


##################################################
def duration_fn(traj):
    distance = sum(np.linalg.norm(np.array(q2) - np.array(q1)) for q1, q2 in zip(traj, traj[1:]))
    return distance / DISTANCE_PER_TIME


def distance_fn(q1, q2):
    ord = 1  # 1 | 2
    return MOVE_COST + COST_PER_DIST * np.linalg.norm(q2 - q1, ord=ord)


def collision_test(b1, p1, b2, p2):
    if b1 == b2:
        return False
    return interval_overlap(get_block_interval(b1, p1), get_block_interval(b2, p2))


def movable_object_test(t, b, p):
    if b == "ghost":
        return False 
    box_interval = get_block_interval(b, p)
    traj = [np.array([200, 200]), np.array([200, 150])]
    for x in t:
        if tuple(p) == tuple(x):
            return False
        #robot_interval = get_robot_interval(q)
        #if interval_overlap(box_interval, robot_interval):
            #return True
    return False 

def get_region_test(regions):
    def test(b, p, r):
        return interval_contains(regions[r], get_block_interval(b, p))

    return test


def sample_region(b, region):
    lower, upper = np.array(region, dtype=float)
    x1, y1 = lower[0], lower[1]
    x2, y2 = upper[0], upper[1]
    x = np.random.uniform(x1 + 40, x2 - 40)
    y = np.random.uniform(y1 + 40, y2 - 40)

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


def sample_line(segment, step_size=0.25):
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
    upper = np.array([570, 570])
    return np.random.random(2) * (upper - lower) + lower


samples = []


def get_sample_fn(obstacles, **kwargs):
    collision_fn = get_collision_fn(obstacles)

    def region_gen():
        while True:
            q = np.array(sample_box())
            if collision_fn(q):
                continue
            samples.append(q)
            return q

    return region_gen


def get_collision_fn(obstacles):
    def collision_fn(q):
        if robot_collides(q, obstacles):
            return True
        return False

    return collision_fn


def plan_motion(q1, q2, obstacles):
    t  = [q1, q2]
    return (t, )
    if len(obstacles) == 0:
        x1, y1 = q1 
        if y1 > 100:
            t = [q1, q2]
            return (t,)
        else:
            t = [q1, q2]
            return (t,)
    extend_fn = get_extend_fn(obstacles)
    sample_fn = get_sample_fn(obstacles)
    collision_fn = get_collision_fn(obstacles)
    path = lazy_prm(q1, q2, sample_fn, extend_fn, collision_fn, num_samples=10000)
    return (path[0],)

##################################################
TAMPProblem = namedtuple(
    "TAMPProblem", ["initial", "regions", "obstacles", "goal_conf", "goal_regions"]
)

GOAL_NAME = "toilet"
TABLE_NAME = "table"

INITIAL_CONF = np.array([200, 350])
GOAL_CONF = np.array([200, 50])

REGIONS = {
'env':[(0, 0), (400, 400)],
'goal': [(150, 0), (250, 50)],
'fakeenv': [(0, 145), (400, 400)],
'start':[(150, 350), (250, 400)],
}


# OBSTACLES = [[(0, 150), (150, 160)], [(250, 0), (255, 200)]]
envs = list(REGIONS.keys())
# envs.remove(GOAL_NAME)
ENVIRONMENT_NAMES = envs
OBSTACLES = [[(0, 0), (140, 140)], [(260, 0), (400, 140)], [(0, 260), (140, 400)],[(260, 260), (400, 400)]]
#OBSTACLES = []

def tight(n_blocks=2, n_goals=2, n_robots=1):
    confs = [INITIAL_CONF, np.array([-1, 1]) * INITIAL_CONF]
    robots = ["r{}".format(x) for x in range(n_robots)]
    initial_confs = dict(zip(robots, confs))
    poses = [(180, 120), (230, 130), (200, 70)]
    blocks = ["A","ghost"]
    initial = TAMPState(initial_confs, {}, dict(zip(blocks, poses)))
    goal_regions = {"A" : "fakeenv", "B" : "fakeenv", "ghost" : "goal"}  # GROUND_NAME

    return TAMPProblem(initial, REGIONS, OBSTACLES, GOAL_CONF, goal_regions)


PROBLEMS = [tight]

def prune_duplicates(traj):
    # TODO: could use the more general sparcify function
    new_traj = [traj[0]]
    for conf in traj[1:]:
        if 0 < np.linalg.norm(np.array(conf) - np.array(new_traj[-1])):
            new_traj.append(conf)
    return new_traj

def get_value_at_time(traj, fraction):
    waypoints = prune_duplicates(traj)
    if len(waypoints) == 1:
        return waypoints[0]
    distances = [0.] + [np.linalg.norm(np.array(q2) - np.array(q1))
                        for q1, q2 in zip(waypoints, waypoints[1:])]
    cum_distances = np.cumsum(distances)
    cum_fractions = np.minimum(cum_distances / cum_distances[-1], np.ones(cum_distances.shape))
    index = np.digitize(fraction, cum_fractions, right=False)
    if index == len(waypoints):
        index -= 1
    waypoint_fraction = (fraction - cum_fractions[index - 1]) / (cum_fractions[index] - cum_fractions[index - 1])
    waypoint1, waypoint2 = np.array(waypoints[index - 1]), np.array(waypoints[index])
    conf = (1 - waypoint_fraction) * waypoint1 + waypoint_fraction * waypoint2
    #if np.isnan(conf).any():
    #    print(distances, fraction)
    #    raw_input()
    return conf

def update_state(state, action, t):
    robot_confs, holding, block_poses = state
    name, args, start, duration = action
    fraction = float(t) / duration
    fraction = max(0, min(fraction, 1))
    assert 0 <= fraction <= 1
    threshold = 0.5
    if name == 'move':
        robot, _, traj, _ = args
        robot_confs[robot] = get_value_at_time(traj, fraction)
    elif name == 'pick':
        robot, block, pose, grasp, conf = args[:5]
        traj = [conf, pose - grasp]
        if fraction < threshold:
            robot_confs[robot] = get_value_at_time(traj, fraction / threshold)
        else:
            holding[robot] = (block, grasp)
            block_poses.pop(block, None)
            robot_confs[robot] = get_value_at_time(traj[::-1], (fraction - threshold) / (1 - threshold))
    elif name == 'place':
        robot, block, pose, grasp, conf = args[:5]
        traj = [conf, pose - grasp]
        if fraction < threshold:
            robot_confs[robot] = get_value_at_time(traj, fraction / threshold)
        else:
            holding.pop(robot, None)
            block_poses[block] = pose
            robot_confs[robot] = get_value_at_time(traj[::-1], (fraction - threshold) / (1 - threshold))
    elif name == 'cook':
        # TODO: update the object color
        pass
    else:
        raise ValueError(name)
    return TAMPState(robot_confs, holding, block_poses)