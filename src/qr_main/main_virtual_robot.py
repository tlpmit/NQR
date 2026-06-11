from qr_api.virtual_robot_interfaces import VirtualRobotModule
from qr_main.main import QRMain, PlanFailedException, PlanFinishedException
from qr_api.policy_interfaces import PolicyModule
from qr_api.belief_interfaces import BeliefModule
from qr_api.sensor_typing import Observation
from qr_api.policy_typing import Action, ActionRV
from qr_utils.misc_utils import pickle_to_QR_log_dir, load_from_pkl_dir
from qr_run.qr_config import QRSystemConfig
from qr_utils.traceFile import debug_add, pause_add

class QRMainVR(QRMain):
    def __init__(self, virtual_robot : VirtualRobotModule, qr_configuration: QRSystemConfig) -> None:
        self._control_interfaces = virtual_robot.control_modules if virtual_robot else {}
        self._sensor_interfaces = virtual_robot.sensor_modules if virtual_robot else {}
        self._virtual_robot = virtual_robot
        super().__init__(self._sensor_interfaces, self._control_interfaces)
        assert not (qr_configuration.write_to_pkl_path and 
                    qr_configuration.run_from_pkl_path), 'Cannot write to pkl and run from pkl at the same time.'
        self.write_to_pkl_path = qr_configuration.write_to_pkl_path
        self.run_from_pkl_path = qr_configuration.run_from_pkl_path
        self.run_from_pkl_n_iterations = qr_configuration.run_from_pkl_n_iterations
        debug_add(qr_configuration.terminal_tags)
        pause_add(qr_configuration.pause_tags)

    def main(self):
        assert self._belief_module is not None, 'The belief module is not set.'
        assert self._policy_module is not None, 'The policy module is not set.'
        if self.run_from_pkl_path:
            return self.main_from_pkl()

        # Allow the process to start by getting an observation
        observation = self._policy_module.get_observation_reset()
        if self.write_to_pkl_path:
            pickle_to_QR_log_dir(observation, 'observation', increment_num=True)

        self._belief_module.reset(observation)

        try:

            try:
                self._policy_module.reset(observation)
            except PlanFailedException as e:
                raise e

            while True:
                try:
                    action = self._policy_module.step()
                except PlanFinishedException:
                    return
                except PlanFailedException as e:
                    raise e

                action_rv = action.execute(self._belief_module, self._virtual_robot)
                observation = self._policy_module.get_observation_step(action)

                if self.write_to_pkl_path:
                    pickle_to_QR_log_dir(action.pkl_copy(), 'action', increment_num=True)
                    pickle_to_QR_log_dir(action_rv, 'action_rv', increment_num=False)
                    pickle_to_QR_log_dir(observation, 'observation', increment_num=False)

                self._belief_module.update(action, action_rv, observation)
                self._policy_module.update(action, action_rv, observation) 
        except Exception as e:
            print(e)
            raise
        finally:
            self._policy_module.shutdown()
            self._virtual_robot.shutdown()   

    def main_from_pkl(self):
        observation = load_from_pkl_dir(self.run_from_pkl_path, 'observation', increment_num=True)
        if observation is None:
            raise ValueError('No observation found in pkl file.')

        self._belief_module.reset(observation)
        try:
            self._policy_module.reset(observation)
        except PlanFailedException as e:
            raise e
        
        robot = self.policy_module.planning_bel.phys.robot

        i = 0
        n_iterations = self.run_from_pkl_n_iterations
        while n_iterations is None or i < n_iterations:
            action = load_from_pkl_dir(self.run_from_pkl_path, 'action', increment_num=True)
            if action is None:
                break
            action.policy_module = self._policy_module
            action_rv = load_from_pkl_dir(self.run_from_pkl_path, 'action_rv', increment_num=False)
            observation = load_from_pkl_dir(self.run_from_pkl_path, 'observation', increment_num=False)

            # Shouldn't need this!
            camera_params = list(robot.sensors.values())[0].params
            for im in observation['images']:
                if im.camera_params is None:
                    print("Fixing missing camera parameters in observation image.")
                    im.camera_params = camera_params

            self._belief_module.update(action, action_rv, observation)
            self._policy_module.update(action, action_rv, observation)
            i += 1
        
        # Now call the policy once
        try:
            action = self._policy_module.step()
        except PlanFinishedException:
            return
        except PlanFailedException as e:
            raise e
        finally:
            self._policy_module.shutdown()

class PolicyModuleVR(PolicyModule):
    def __init__(self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule, problem: dict):
        super().__init__(belief_module, virtual_robot.control_modules)
        self._virtual_robot = virtual_robot    
        self._problem = problem
    
    def reset(self, observation: Observation):
        pass

    def step(self) -> Action:
        pass

    def update(self, action: Action, action_rv: ActionRV, observation: Observation):
        pass

    def get_observation_reset(self) -> Observation:
        pass
    
    def get_observation_step(self) -> Observation:
        observation = self._virtual_robot.get_observation()
        return observation
    
class ActionVR(Action):
    def execute(self, belief_module: BeliefModule, virtual_robot: VirtualRobotModule) -> None:
        pass

    def update_belief_before(self, belief_module: BeliefModule, action_rv: ActionRV, observation: Observation):
        pass

    def update_belief_after(self, belief_module: BeliefModule, action_rv: ActionRV, observation: Observation):
        pass    