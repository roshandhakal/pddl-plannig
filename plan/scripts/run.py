#!/usr/bin/env python

"""
This file contains code to run continous TAMP
example on 2D environment (top down view of Environment).
The main purpose of this file is just to see a running
example. Almost all codes except for some modification
are taken from the example of continous_tamp in PDDLStream.
There is a test code to check if the environment we created
has successful pddlstream implementation.

For more info on PDDLStream Continous TAMP:
https://github.com/caelan/pddlstream/blob/main/examples/continuous_tamp/

"""
from __future__ import print_function
import os
import random
import numpy as np
import time
from itertools import product
from pddlstream.algorithms.meta import solve, create_parser
from examples.continuous_tamp.optimizer.optimizer import (
    cfree_motion_fn,
    get_optimize_fn,
)
from plan.environments.blockworld.primitives import (
    GOAL_CONF,
    INITIAL_CONF,
    get_pose_gen,
    collision_test,
    movable_object_test,
    get_region_test,
    plan_motion,
    PROBLEMS,
    GRASP,
    ENVIRONMENT_NAMES,
    distance_fn,
    MOVE_COST,
    duration_fn,
    update_state,
)
from examples.continuous_tamp.primitives import (
    inverse_kin_fn,
    get_random_seed,
)
from pddlstream.algorithms.downward import get_cost_scale
from pddlstream.algorithms.constraints import PlanConstraints, WILD
from pddlstream.algorithms.visualization import VISUALIZATIONS_DIR
from pddlstream.language.external import (
    defer_shared,
    get_defer_all_unbound,
    get_defer_any_unbound,
)
from pddlstream.language.constants import (
    And,
    Equal,
    PDDLProblem,
    TOTAL_COST,
    print_solution,
    Or,
    Output,
)
from pddlstream.language.function import FunctionInfo
from pddlstream.language.generator import from_gen_fn, from_list_fn, from_test, from_fn
from pddlstream.language.stream import StreamInfo
from pddlstream.language.temporal import get_end, compute_duration, retime_plan
from pddlstream.utils import (
    ensure_dir,
    safe_rm_dir,
    user_input,
    read,
    INF,
    get_file_path,
    str_from_object,
    sorted_str_from_list,
    implies,
    inclusive_range,
    Profiler,
)


##################################################
def create_problem(tamp_problem, hand_empty=False, manipulate_cost=1.0):
    initial = tamp_problem.initial
    assert not initial.holding
    init = [Equal(("Cost",), manipulate_cost), Equal((TOTAL_COST,), 0)]
    boxes = ["box1", "box2", "box3", "box4"]
    for b in initial.block_poses.keys():
        for r in tamp_problem.regions:
            if r not in boxes:
                init.append(("Placeable", b, r))

    init += [("Obstacle", tamp_problem.obstacles)]

    for b, p in initial.block_poses.items():
        init += [
            ("Block", b),
            ("Pose", b, p),
            ("AtPose", b, p),
        ]

    goal_literals = []
    for b, r in tamp_problem.goal_regions.items():
        if isinstance(r, np.ndarray):
            init += [("Pose", b, r)]
            goal_literals += [("AtPose", b, r)]
        else:
            blocks = [b] if isinstance(b, str) else b
            regions = [r] if isinstance(r, str) else r
            conditions = []
            for body, region in product(blocks, regions):
                init += [("Region", region), ("Placeable", body, region)]
                conditions += [("In", body, region)]
            goal_literals.append(Or(*conditions))

    for r, q in initial.robot_confs.items():
        init += [
            ("Robot", r),
            ("CanMove", r),
            ("Conf", q),
            ("AtConf", r, q),
            ("HandEmpty", r),
        ]
        if hand_empty:
            goal_literals += [("HandEmpty", r)]
        
    #goal_literals += [("AtConf", "r0", GOAL_CONF)]
    goal = And(*goal_literals)

    return init, goal


def pddlstream_from_tamp(
    tamp_problem, use_stream=True, use_optimizer=False, collisions=True
):
    domain_pddl = read(get_file_path(__file__, "../domain.pddl"))
    external_paths = []
    if use_stream:
        external_paths.append(get_file_path(__file__, "../stream.pddl"))
    external_pddl = [read(path) for path in external_paths]

    constant_map = {}
    stream_map = {
        "s-grasp": from_fn(lambda b: Output(GRASP)),
        "s-region": from_gen_fn(get_pose_gen(tamp_problem.regions)),
        "s-ik": from_fn(inverse_kin_fn),
        "s-motion": from_fn(plan_motion),
        "t-region": from_test(get_region_test(tamp_problem.regions)),
        "t-movableboxfree": from_test(
            lambda *args: implies(collisions, not movable_object_test(*args))
        ),
        "t-cfree": from_test(
            lambda *args: implies(collisions, not collision_test(*args))
        ),
        "dist": distance_fn,
        "duration": duration_fn,
    }
    if use_optimizer:
        stream_map.update(
            {
                "gurobi": from_list_fn(
                    get_optimize_fn(tamp_problem.regions, collisions=collisions)
                ),
                "rrt": from_fn(cfree_motion_fn),
            }
        )
    init, goal = create_problem(tamp_problem)
    return PDDLProblem(domain_pddl, constant_map, external_pddl, stream_map, init, goal)


##################################################


def display_plan(
    tamp_problem, plan, display=True, save=False, time_step=0.005, sec_per_step=1e-20
):
    from plan.environments.blockworld.viewer import ContinuousTMPViewer
    COLORS = ['red', 'orange', 'blue']

    if save:
        example_name = "continuous_tamp"
        directory = os.path.join(VISUALIZATIONS_DIR, "{}/".format(example_name))
        safe_rm_dir(directory)
        ensure_dir(directory)

    colors = dict(zip(sorted(tamp_problem.initial.block_poses.keys()), COLORS))
    viewer = ContinuousTMPViewer(
        tamp_problem.regions, tamp_problem.obstacles,  title="Continuous TAMP"
    )
    
    state = tamp_problem.initial
    print()
    print(state)
    duration = compute_duration(plan)
    real_time = None if sec_per_step is None else (duration * sec_per_step / time_step)
    print(
        "Duration: {} | Step size: {} | Real time: {}".format(
            duration, time_step, real_time
        )
    )
    viewer.draw_state(state, colors)
    if display:
        user_input("Start?")
    if plan is not None:
        for t in inclusive_range(0, duration, time_step):
            for action in plan:
                if action.start <= t <= get_end(action):
                    update_state(state, action, t - action.start)
            print("t={} | {}".format(t, state))
            viewer.draw_state(state, colors)
            if save:
                viewer.save(os.path.join(directory, "t={}".format(t)))
            if display:
                if sec_per_step is None:
                    user_input("Continue?")
                else:
                    time.sleep(sec_per_step)
    if display:
        user_input("Finish?")

    return state


##################################################


TIGHT_SKELETON = [
    ("move", ["r0", "?q0", WILD, "?q1"]),
    ("pick", ["r0", "B", "?p0", "?g0", "?q1"]),
    ("move", ["r0", "?q1", WILD, "?q2"]),
    ("place", ["r0", "B", "?p1", "?g0", "?q2"]),
    ("move", ["r0", "?q2", WILD, "?q3"]),
    ("pick", ["r0", "A", "?p2", "?g1", "?q3"]),
    ("move", ["r0", "?q3", WILD, "?q4"]),
    ("place", ["r0", "A", "?p3", "?g1", "?q4"]),
    ("move", ["r0", "?q4", WILD, "?q5"]),
]

MUTEXES = []

##################################################


def set_deterministic():
    random.seed(0)
    np.random.seed(0)


def initialize(parser):
    parser.add_argument(
        "-c", "--cfree", action="store_true", help="Disables collisions"
    )
    parser.add_argument(
        "-d",
        "--deterministic",
        action="store_true",
        help="Uses a deterministic sampler",
    )
    parser.add_argument("-t", "--max_time", default=1000, type=int, help="The max time")
    parser.add_argument(
        "-n", "--number", default=2, type=int, help="The number of blocks"
    )
    parser.add_argument(
        "-p", "--problem", default="tight", help="The name of the problem to solve"
    )
    parser.add_argument(
        "-v", "--visualize", action="store_true", help="Visualizes graphs"
    )

    args = parser.parse_args()
    print("Arguments:", args)
    np.set_printoptions(precision=2)
    set_deterministic()
    #print("Random seed:", get_random_seed())

    problem_from_name = {fn.__name__: fn for fn in PROBLEMS}
    if args.problem not in problem_from_name:
        raise ValueError(args.problem)
    print("Problem:", args.problem)
    problem_fn = problem_from_name[args.problem]
    tamp_problem = problem_fn()
    print(tamp_problem)
    return tamp_problem, args


def dump_pddlstream(pddlstream_problem):
    print("Initial:", sorted_str_from_list(pddlstream_problem.init))
    print("Goal:", str_from_object(pddlstream_problem.goal))


##################################################

def main():
    parser = create_parser()
    parser.add_argument("-g", "--gurobi", action="store_true", help="Uses gurobi")
    parser.add_argument(
        "-o", "--optimal", action="store_true", help="Runs in an anytime mode"
    )
    parser.add_argument(
        "-s",
        "--skeleton",
        action="store_true",
        help="Enforces skeleton plan constraints",
    )
    tamp_problem, args = initialize(parser)

    defer_fn = defer_shared
    stream_info = {
        "s-region": StreamInfo(defer_fn=defer_fn),
        "s-grasp": StreamInfo(defer_fn=defer_fn),
        "s-ik": StreamInfo(defer_fn=get_defer_all_unbound(inputs="?g")),
        "s-motion": StreamInfo(defer_fn=get_defer_any_unbound()),
        "t-cfree": StreamInfo(
            defer_fn=get_defer_any_unbound(), eager=False, verbose=False
        ),
        "t-movableboxfree": StreamInfo(
            defer_fn=get_defer_any_unbound(), eager=False, verbose=False
        ),
        "t-region": StreamInfo(eager=True, p_success=0),
        "dist": FunctionInfo(
            eager=False,
            defer_fn=get_defer_any_unbound(),
            opt_fn=lambda q1, q2: MOVE_COST,
        ),
        "gurobi-cfree": StreamInfo(eager=False, negate=True),
    }

    hierarchy = []
    skeletons = [TIGHT_SKELETON] if args.skeleton else None
    assert implies(args.skeleton, args.problem == "tight")
    max_cost = INF
    constraints = PlanConstraints(skeletons=skeletons, exact=True, max_cost=max_cost)
    replan_actions = set()
    pddlstream_problem = pddlstream_from_tamp(
        tamp_problem,
        collisions=not args.cfree,
        use_stream=not args.gurobi,
        use_optimizer=args.gurobi,
    )

    dump_pddlstream(pddlstream_problem)

    success_cost = INF
    planner = "max-astar"
    effort_weight = 1.0 / get_cost_scale()

    with Profiler(field="cumtime", num=20):
        solution = solve(
            pddlstream_problem,
            algorithm=args.algorithm,
            constraints=constraints,
            stream_info=stream_info,
            replan_actions=replan_actions,
            planner=planner,
            max_planner_time=INF,
            hierarchy=hierarchy,
            max_time=INF,
            max_iterations=30,
            debug=False,
            verbose=True,
            unit_costs=args.unit,
            success_cost=success_cost,
            unit_efforts=True,
            effort_weight=effort_weight,
            search_sample_ratio=1,
            visualize=args.visualize,
        )

    print_solution(solution)
    plan, cost, evaluations = solution
    if plan is not None:
        display_plan(tamp_problem, retime_plan(plan))


if __name__ == "__main__":
    main()