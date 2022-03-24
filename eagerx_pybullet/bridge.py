# Pybullet imports
from typing import Optional, Dict, Union, List
import os

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64
from genpy.message import Message

# RX IMPORTS
from eagerx.core.constants import process, ERROR
import eagerx.core.register as register
from eagerx.core.entities import Bridge
from eagerx.core.specs import BridgeSpec
from eagerx_pybullet.pybullet_world import World
from eagerx_pybullet.pybullet_robot import URDFBasedRobot

os.environ["PYBULLET_EGL"] = "1"
# ^^^^ before importing pybullet_gym
try:
    if os.environ["PYBULLET_EGL"]:
        import pkgutil
except BaseException:
    pass

try:
    import pybullet
    import pybullet_data
    from pybullet_utils import bullet_client
except ImportError as e:
    from gym import error

    raise error.DependencyNotInstalled("{}. (HINT: you need to install PyBullet)".format(e))


class PybulletBridge(Bridge):
    @staticmethod
    @register.spec("PybulletBridge", Bridge)
    def spec(
        spec: BridgeSpec,
        rate,
        process: Optional[int] = process.NEW_PROCESS,
        is_reactive: Optional[bool] = True,
        real_time_factor: Optional[float] = 0,
        simulate_delays: Optional[bool] = True,
        log_level: Optional[int] = ERROR,
        world: Optional[str] = None,
        gui: bool = True,
        num_substeps: int = 1,
        num_solver_iterations: int = 5,
        contact_erp: float = 0.9,
    ):
        """
        Spec of the PybulletBridge

        :param spec: Not provided by the user.
        :param rate: Rate of the bridge
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: BRIDGE, 3: EXTERNAL}
        :param is_reactive: Run reactive or async
        :param real_time_factor: simulation speed. 0 == "as fast as possible".
        :param simulate_delays: Boolean flag to simulate delays.
        :param log_level: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}
        :return: BridgeSpec
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(PybulletBridge)

        # Modify default bridge params
        spec.config.rate = rate
        spec.config.process = process
        spec.config.is_reactive = is_reactive
        spec.config.real_time_factor = real_time_factor
        spec.config.simulate_delays = simulate_delays
        spec.config.log_level = log_level
        spec.config.color = "magenta"

        # Add custom params
        spec.config.world = world if world else "%s/%s.urdf" % (pybullet_data.getDataPath(), "plane")
        spec.config.gui = gui
        spec.config.num_substeps = num_substeps
        spec.config.num_solver_iterations = num_solver_iterations
        spec.config.contact_erp = contact_erp

    def initialize(self, world, gui, num_substeps, num_solver_iterations, contact_erp):
        # Initialize simulator
        self._p, self.physics_client_id = self._start_simulator(gui)

        world = World(
            self._p,
            gravity=9.81,
            world_file=world,
            timestep=1 / self.rate,
            frame_skip=num_substeps,
            num_solver_iterations=num_solver_iterations,
            contact_erp=contact_erp,
        )
        self.simulator = dict(client=self._p, world=world, robots={})

    def _start_simulator(self, gui):
        if gui:
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            p = bullet_client.BulletClient()

        physics_client_id = p._client
        p.resetSimulation()
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        # optionally enable EGL for faster headless rendering
        try:
            if os.environ["PYBULLET_EGL"]:
                con_mode = p.getConnectionInfo()["connectionMethod"]
                if con_mode == p.DIRECT:
                    egl = pkgutil.get_loader("eglRenderer")
                    if egl:
                        p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
                    else:
                        p.loadPlugin("eglRendererPlugin")
                    #
                    # todo: STRANGE! Must pre-render atleast nun_runs=2 images, else crash in _camera_callback(...)
                    # self._test_fps_rendering(p, physics_client_id, num_runs=100)
        except BaseException:
            pass

        return p, physics_client_id

    @register.bridge_config(
        urdf=None, basePosition=[0, 0, 0], baseOrientation=[0, 0, 0, 0], fixed_base=True, self_collision=True
    )
    def add_object(self, config, bridge_config, node_params, state_params):
        obj_name = config["name"]
        entity_id = config["entity_id"]

        # add objects to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo(f'Adding object "{obj_name}" of type "{entity_id}" to the simulator.')

        # Add objects
        if bridge_config["urdf"]:
            self.simulator["robots"][obj_name] = URDFBasedRobot(
                self._p,
                model_urdf=bridge_config["urdf"],
                robot_name=obj_name,
                basePosition=bridge_config["basePosition"],
                baseOrientation=bridge_config["baseOrientation"],
                fixed_base=bridge_config["fixed_base"],
                self_collision=bridge_config["self_collision"],
            )
        else:  # if no urdf is provided, create dummy robot.
            self.simulator["robots"][obj_name] = None

    def pre_reset(self):
        pass

    @register.states()
    def reset(self):
        pass

    @register.outputs(tick=UInt64)
    def callback(self, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        self.simulator["world"].step()

    def shutdown(self) -> None:
        self._p.disconnect()
