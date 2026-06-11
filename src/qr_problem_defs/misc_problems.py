# Can't handle Jiayuan's fancy templated "urdf" files
# domain_qddl_filename = osp.join(osp.dirname(__file__), 'simple-cube-in-box-domain.qddl')
# problem_qddl_filename = osp.join(osp.dirname(__file__), 'simple-cube-in-box-problem.qddl')
# extra_paths = ['/Users/lpk/lpkmac/git/Concepts/concepts/assets']
from pathlib import Path

from qr_problem_defs.problem_def import QRProblem

qr_path = Path(__file__).parent.parent.parent
qr_src_path = qr_path / "src"
hpn_path = Path(__file__).parent.parent.parent.parent / "HPN" / "HPN"
concepts_path = Path(__file__).parent.parent.parent / "Concepts" / "concepts"
hpn_domains_path = hpn_path / "Domains"
crow_asset_path = concepts_path / "assets"
qr_assets_path = qr_path / "assets"
hpn_models_path = qr_assets_path / "hpn_models"
hpn_crow_test_asset_path = hpn_domains_path / "fully_observed_tamp/tests/crow_tests"


# from qr_utils import GetQRSourcePath
# def ex(f): return GetQRSourcePath() / f
def ex(f):
    return qr_src_path / f


extra_crow_paths = [str(crow_asset_path), str(hpn_crow_test_asset_path)]
extra_qr_paths = [
    str(qr_assets_path),
    str(qr_assets_path / "open-world-tamp"),
    str(hpn_models_path),
]

problem_crow_0 = QRProblem(
    goal="(and (exists ?x (and (color ?x magenta) (holding ?x))))",
    domain=f"{hpn_crow_test_asset_path}/pick-place-domain.qddl",
    world=f"{hpn_crow_test_asset_path}/pick-place-problem0.qddl",
    belief=f"{hpn_crow_test_asset_path}/pick-place-belief.qddl",
    assets=extra_crow_paths,
    partially_observed=False,
    robot="panda",
    virtual_robot="Roboverse_Sim",
)  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'

spot_1 = QRProblem(
    goal="(and (exists ?x (and (color ?x magenta) (holding ?x))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_no_top.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_1_drake = QRProblem(
    goal="(and (exists ?x (and (color ?x magenta) (holding ?x))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_no_top.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Spot_Drake_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_2 = QRProblem(
    goal="(and (exists ?x (and (color ?x magenta) (holding ?x))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    # world = ex("qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_two_tables.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_1 = QRProblem(
    goal="(and (exists ?x (and (color ?x magenta) (holding ?x))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_no_top.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_1_move_base = QRProblem(
    goal="(and (exists ?x (and (color ?x magenta) (holding ?x))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_no_top.pddl"
    ),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_far = QRProblem(
    goal="(and (exists ?x (and (color ?x magenta) (holding ?x))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_no_top_far.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)


def on_goal(attr1, val1, attr2, val2):
    return f"(and (exists ?x (exists ?y (and (on ?x ?y) ({attr1} ?x {val1}) ({attr2} ?y {val2})))))"


def holding_goal(attr, val):
    return f"(and (exists ?x (and ({attr} ?x {val}) (holding ?x))))"


red_on_green = on_goal("color", "red", "color", "green")
yellow_on_green = on_goal("color", "yellow", "color", "green")
red_on_blue = on_goal("color", "red", "color", "blue")
green_on_green = on_goal("color", "green", "color", "green")
green_on_blue = on_goal("color", "green", "color", "blue")
holding_green = holding_goal("color", "green")
holding_yellow = holding_goal("color", "yellow")
holding_magenta = holding_goal("color", "magenta")
holding_red = holding_goal("color", "red")
holding_blue = holding_goal("color", "blue")
green_on_magenta = on_goal("color", "green", "color", "magenta")

spot_real = QRProblem(
    goal=holding_green,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=None,
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Topaz",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_pick_place = QRProblem(
    goal=green_on_magenta,
    domain=ex("qr_domains/fully_observed_tamp/tests/colored_objects/domain.pddl"),
    world=ex(
        "qr_domains/fully_observed_tamp/tests/colored_objects/problem_spot_book.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_pick_place_occluder = QRProblem(
    goal=green_on_magenta,
    domain=ex("qr_domains/fully_observed_tamp/tests/colored_objects/domain.pddl"),
    world=ex(
        "qr_domains/fully_observed_tamp/tests/colored_objects/problem_spot_book_occluder.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_holding_green = QRProblem(
    goal=holding_green,
    domain=ex("qr_domains/fully_observed_tamp/tests/colored_objects/domain.pddl"),
    world=ex(
        "qr_domains/fully_observed_tamp/tests/colored_objects/problem_spot_book.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)


ruby_pick_place_no_move_base = QRProblem(
    goal=green_on_magenta,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/fully_observed_tamp/tests/colored_objects/problem_green_on_book_table_close.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_pick_place_move_base = QRProblem(
    # goal=green_on_magenta,
    goal=red_on_green,
    domain=ex("qr_domains/fully_observed_tamp/tests/colored_objects/domain.pddl"),
    world=ex(
        "qr_domains/fully_observed_tamp/tests/colored_objects/problem_green_on_book.pddl"
    ),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_mj_pick_place_move_base = QRProblem(
    goal=red_on_green,
    domain=ex("qr_domains/fully_observed_tamp/tests/colored_objects/domain.pddl"),
    world=ex(
        "qr_domains/fully_observed_tamp/tests/colored_objects/problem_green_on_book.pddl"
    ),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Ruby_Mujoco_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_drake_pick_place_move_base = QRProblem(
    goal=red_on_green,
    domain=ex("qr_domains/fully_observed_tamp/tests/colored_objects/domain.pddl"),
    world=ex(
        "qr_domains/fully_observed_tamp/tests/colored_objects/problem_green_on_book.pddl"
    ),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Ruby_Drake_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_real = QRProblem(
    # goal = yellow_on_green,
    goal=green_on_blue,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=None,
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Ruby",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_real_move_base_holding = QRProblem(
    goal=holding_green,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=None,
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Ruby",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_real_move_base_pick_place = QRProblem(
    goal=green_on_blue,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=None,
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Ruby",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_occluded_grape_drake = QRProblem(
    goal="(and (exists ?g (and (color ?g green) (holding ?g))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_occluded_grape.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Spot_Drake_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_occluded_grape = QRProblem(
    goal="(and (exists ?g (and (color ?g green) (holding ?g))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_occluded_grape.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

ruby_occluded_grape = QRProblem(
    goal="(and (exists ?g (and (color ?g green) (holding ?g))))",
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_occluded_grape.pddl"
    ),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

spot_pick_place_harder = QRProblem(
    goal=on_goal("color", "magenta", "color", "blue"),
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_two_tables.pddl"
    ),
    belief=ex("qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="spot",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)


ruby_pick_place_harder = QRProblem(
    goal=on_goal("color", "magenta", "color", "blue"),
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/problem_shelf_qr_two_tables.pddl"
    ),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="rainbow",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

panda_pick_place = QRProblem(
    # goal = '(and (exists ?x (and (description ?x "banana") (holding ?x))))',
    goal='(and (exists ?x (and (color ?x "yellow") (holding ?x))))',
    domain=ex("qr_domains/mobile_po_tamp/tests/panda_table/domain_qr.pddl"),
    world=ex("qr_domains/mobile_po_tamp/tests/panda_table/problem_ycb.pddl"),
    belief=ex("qr_domains/mobile_po_tamp/tests/panda_table/panda_empty_belief.pddl"),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="panda_droid",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

panda_droid_pick_place = QRProblem(
    goal=red_on_blue,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex(
        "qr_domains/mobile_po_tamp/tests/panda_table//problem_droid_red_on_blue.pddl"
    ),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/panda_table/panda_droid_empty_belief.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="panda_droid",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

panda_droid_pick_place_harder = QRProblem(
    goal=red_on_blue,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex("qr_domains/mobile_po_tamp/tests/panda_table//problem_droid_harder.pddl"),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/panda_table/panda_droid_empty_belief.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="panda_droid",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

panda_droid_pick_place_shield = QRProblem(
    goal=red_on_blue,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=ex("qr_domains/mobile_po_tamp/tests/panda_table/problem_droid_shield.pddl"),
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/panda_table/panda_droid_empty_belief.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="panda_droid",
    virtual_robot="Roboverse_Sim",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)

panda_droid_real = QRProblem(
    goal=holding_blue,
    domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
    world=None,
    belief=ex(
        "qr_domains/mobile_po_tamp/tests/panda_table/panda_droid_empty_belief.pddl"
    ),
    assets=extra_qr_paths,
    partially_observed=True,
    robot="panda_droid",
    virtual_robot="PandaDroid",  # 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'
)


def make_sim_mobile_pick_place_with_exprs(x_expr, y_expr, robot):
    if robot == "rainbow":
        belief_path = "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
    elif robot == "panda":
        belief_path = (
            "qr_domains/mobile_po_tamp/tests/panda_table/panda_droid_empty_belief.pddl"
        )
    elif robot == "spot":
        belief_path = (
            "qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"
        )
    else:
        raise RuntimeError("Unknown robot")

    goal = f'(and (exists ?x (exists ?y (and (description ?x "{x_expr}") (description ?y "{y_expr}") (on ?x ?y)))))'

    return QRProblem(
        goal=goal,
        domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
        world=ex(
            "qr_domains/fully_observed_tamp/tests/colored_objects/problem_green_on_book.pddl"
        ),
        assets=extra_qr_paths,
        belief=ex(belief_path),
        partially_observed=True,
        robot=robot,
        virtual_robot="Roboverse_Sim",
    )


def make_real_pick_place_with_exprs(x_expr, y_expr, robot):
    if robot == "rainbow":
        belief_path = "qr_domains/mobile_po_tamp/tests/mobile_table/rainbow_empty_belief_move_base.pddl"
        virtual_robot = "Ruby"
    elif robot == "panda":
        belief_path = (
            "qr_domains/mobile_po_tamp/tests/panda_table/panda_droid_empty_belief.pddl"
        )
        virtual_robot = "PandaDroid"
    elif robot == "spot":
        belief_path = (
            "qr_domains/mobile_po_tamp/tests/mobile_table/spot_empty_belief.pddl"
        )
        virtual_robot = "Topaz"
    else:
        raise RuntimeError("Unknown robot")

    goal = f'(and (exists ?x (exists ?y (and (description ?x "{x_expr}") (description ?y "{y_expr}") (on ?x ?y)))))'

    return QRProblem(
        goal=goal,
        domain=ex("qr_domains/mobile_po_tamp/tests/mobile_table/domain_qr.pddl"),
        world=None,
        belief=ex(belief_path),
        partially_observed=True,
        robot=robot,
        virtual_robot=virtual_robot,
    )
