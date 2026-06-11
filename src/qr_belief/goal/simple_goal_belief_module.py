from qr_utils.traceFile import tr, tr_a  # noqa

from qr_api.belief_interfaces import GoalMemoryModule
from qr_api.policy_typing import Action, ActionRV, TransitionPrediction
from qr_api.sensor_typing import Observation


class SimpleGoalMemoryModule(GoalMemoryModule):
    def __init__(self, goal):
        super().__init__()
        self.goal = goal
        # A list of strings describing objects we should be looking out for
        self.object_designators_of_interest = self.extract_object_designators(goal)

    def _reset(self, observation: Observation):
        pass

    def _update(
        self,
        action: Action,
        actionrv: ActionRV,
        transition_prediction: TransitionPrediction,
        observation: Observation,
    ):
        pass

    # TODO: should really use the goal parser that's in the qddl processing code
    @staticmethod
    def extract_object_designators(goal: str) -> list[str]:
        """Return every string that is the second argument of a 'description' form."""
        if not goal or not goal.strip():
            return []

        def tokenize(s):
            tokens = []
            i = 0
            while i < len(s):
                if s[i].isspace():
                    i += 1
                elif s[i] == "(":
                    tokens.append("(")
                    i += 1
                elif s[i] == ")":
                    tokens.append(")")
                    i += 1
                elif s[i] == '"':
                    j = i + 1
                    while j < len(s) and s[j] != '"':
                        j += 1
                    tokens.append(s[i : j + 1])
                    i = j + 1
                else:
                    j = i
                    while j < len(s) and not s[j].isspace() and s[j] not in "()":
                        j += 1
                    tokens.append(s[i:j])
                    i = j
            return tokens

        def parse(tokens, pos):
            if tokens[pos] == "(":
                pos += 1
                items = []
                while tokens[pos] != ")":
                    item, pos = parse(tokens, pos)
                    items.append(item)
                return items, pos + 1
            else:
                return tokens[pos], pos + 1

        def collect(expr):
            if not isinstance(expr, list):
                return []
            results = []
            if expr and expr[0] == "description" and len(expr) >= 3:
                arg = expr[2]
                if isinstance(arg, str) and arg.startswith('"'):
                    results.append(arg[1:-1])
            for item in expr:
                results.extend(collect(item))
            return results

        tokens = tokenize(goal)
        tree, _ = parse(tokens, 0)
        return collect(tree)
