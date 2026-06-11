import time
from qr_api.sensor_interfaces import InputModule
from qr_api.sensor_typing import HumanInstructionReceivedEvent, LanguageInstructionInput
from qr_api.control_interfaces import ControlModuleBase


class DummyInputModule(InputModule):
    pass


class DummyControlModule(ControlModuleBase):
    pass


class SimpleGoalStringInputModule(InputModule):
    EVENTS_GENERATES = [HumanInstructionReceivedEvent]

    def __init__(self):
        super().__init__()

    def set_human_instruction(self, goal_string):
        language_input = LanguageInstructionInput(goal_string, time.time())
        self.emit_event(HumanInstructionReceivedEvent([language_input]))
