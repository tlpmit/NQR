from dataclasses import dataclass


@dataclass
class QRProblem:
    goal: str | None = None
    """The goal of the problem in PDDL format."""

    domain: str | None = None
    """The path to the PDDL domain file."""

    world: str | None = None
    """The path to the PDDL world file."""

    belief: str | None = None
    """The path to the PDDL belief file."""

    assets: list | None = None
    """A list of paths to additional assets needed for the problem."""

    partially_observed: bool | None = None
    """Whether the problem is partially observed or not."""

    robot: str | None = None
    """The type of robot used in the problem. Options are 'spot', 'ruby'."""

    virtual_robot: str | None = None
    """The type of virtual robot used in the problem. Options are 'Spot', 'Ruby', 'Ruby_Drake_Sim', 'Roboverse_Sim'."""

    directives_file: str | None = None
    """The path to the directives file for the problem."""
