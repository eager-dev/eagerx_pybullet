# Pybullet imports
from typing import Optional, Dict, Union, List
import os

# ROS IMPORTS
import rospy
from std_msgs.msg import UInt64, Float32
from genpy.message import Message

# RX IMPORTS
from eagerx.core.constants import process, ERROR
import eagerx.core.register as register
from eagerx.core.entities import Bridge, SpaceConverter
from eagerx.core.specs import BridgeSpec
from eagerx_pybullet.world import World
from eagerx_pybullet.robot import URDFBasedRobot


if int(os.environ.get("PYBULLET_EGL", 0)):
    import pkgutil


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
        rate: float,
        process: Optional[int] = process.NEW_PROCESS,
        is_reactive: Optional[bool] = True,
        real_time_factor: Optional[float] = 0,
        simulate_delays: Optional[bool] = True,
        log_level: Optional[int] = ERROR,
        states: List[str] = None,
        world_fn: Optional[str] = None,
        gui: bool = True,
        gravity: float = -9.81,
        physics_engine_params: Optional[Dict] = None,
    ):
        """A spec to create a PybulletBridge node that interfaces with a pybullet physics server.

        :param spec: Holds the desired configuration in a Spec object.
        :param rate: Rate of the bridge
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: BRIDGE, 3: EXTERNAL}
        :param is_reactive: Run reactive or async
        :param real_time_factor: Simulation speed. 0 == "as fast as possible".
        :param simulate_delays: Boolean flag to simulate delays.
        :param log_level: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}
        :param states: Physics engine parameters that are to be varied over episodes as a form of domain randomization.
                       Currently available: `erp`, `contactERP`, `frictionERP`.
        :param world_fn: A string with syntax `module/WorldFnName` that received `bullet_client` as an argument. The
                         function builds-up the (static) world (i.e. loads urdfs into pybullet). See
                         `eagerx_pybullet.world/empty_world_with_plane` for an example.
        :param gui: Create a GUI connection with 3D OpenGL rendering within the same process space as PyBullet.
        :param gravity: Sets the gravity constant along the y-axis.
        :param physics_engine_params: Parameter keys with their desired value. See the pybullet documentation for more
                                      info on the physics engine parameters:
                                      https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.k37c0drzdf21

                                      .. note:: fixedTimeStep cannot be set, as it is determined by the specified rate of
                                                the bridge. Per default, numSubSteps is set such that simulation steps are
                                                taken at 240 hz. Tune numSubSteps to trade performance over accuracy.

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
        spec.config.states = states if isinstance(states, list) else []

        # Add custom params
        spec.config.world_fn = world_fn
        spec.config.gui = gui
        spec.config.gravity = gravity
        spec.config.physics_engine_params = physics_engine_params if isinstance(physics_engine_params, dict) else None

        # Set space converters for registered physics engine parameters.
        # todo: Default values for erp, contactERP, frictionERP? --> getPhysicsEngineParameters() does not include them...
        spec.states.erp.space_converter = SpaceConverter.make("Space_Float32", 0.005, 0.005, dtype="float32")
        spec.states.contactERP.space_converter = SpaceConverter.make("Space_Float32", 0.005, 0.005, dtype="float32")
        spec.states.frictionERP.space_converter = SpaceConverter.make("Space_Float32", 0.9, 0.9, dtype="float32")

    def initialize(self, world_fn, gui, gravity, physics_engine_params: Dict = None):
        """
        Initializes the bridge to pybullet.

        :param world_fn: A string with syntax `module/WorldFnName` that received `bullet_client` as an argument. The
                         function builds-up the (static) world (i.e. loads urdfs into pybullet). See
                         `eagerx_pybullet.world/empty_world_with_plane` for an example.
        :param gui: Create a GUI connection with 3D OpenGL rendering within the same process space as PyBullet.
        :param gravity: Sets the gravity constant along the y-axis.
        :param physics_engine_params: Parameter keys with their desired value. See the pybullet documentation for more
                                      info on the physics engine parameters:
        """

        # Connect to pybullet
        self._p, self.physics_client_id = self._start_simulator(gui)
        # Initialzize
        world = World(
            self._p,
            gravity=gravity,
            world_fn=world_fn,
            timestep=1 / self.rate,
        )
        # Set physics parameters
        if physics_engine_params:
            assert "fixedTimeStep" not in physics_engine_params, (
                "Cannot set the fixedTimeStep via the physics_engine_params. "
                f"This is determined by the bridge's rate: dt = 1/{self.rate} s."
            )
            self._p.setPhysicsEngineParameter(**physics_engine_params)
        # Create pybullet simulator that will be shared with all EngineStates & EngineNodes (if launched in same process).
        self.simulator = dict(client=self._p, world=world, robots={})

    def _start_simulator(self, gui):
        if gui:
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            # p = bullet_client.BulletClient(pybullet.SHARED_MEMORY, options="-shared_memory_key 1234")
            p = bullet_client.BulletClient()

        physics_client_id = p._client
        p.resetSimulation()
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        # optionally enable EGL for faster headless rendering
        if int(os.environ.get("PYBULLET_EGL", 0)):
            con_mode = p.getConnectionInfo()["connectionMethod"]
            if con_mode == p.DIRECT:
                egl = pkgutil.get_loader("eglRenderer")
                if egl:
                    p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
                else:
                    p.loadPlugin("eglRendererPlugin")

        # Add search path for urdfs
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=p._client)
        return p, physics_client_id

    @register.bridge_config(
        urdf=None,
        basePosition=[0, 0, 0],
        baseOrientation=[0, 0, 0, 0],
        fixed_base=True,
        self_collision=False,
        globalScaling=1.0,
        flags=0,
    )
    def add_object(self, config, bridge_config, node_params, state_params):
        """
        Adds an object to the connected Pybullet physics server.

        :param config: The (agnostic) config of the :class:`~eagerx.core.entities.Object` that is to be added.
        :param bridge_config: The bridge-specific config of the :class:`~eagerx.core.entities.Object` that is to be added.
                              This dict contains the registered parameters:

                              See https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#
                              for all available flags.

                              - **urdf**: A fullpath (ending with .urdf), a key that points to the urdf (xml)string on the
                                rosparam server, or a urdf within pybullet's search path. The `pybullet_data` package is
                                included in the search path.

                              - **basePosition**: Base position of the object [x, y, z].

                              - **baseOrientation**: Base orientation of the object in quaternion [x, y, z, w].

                              - **fixed_base**: Force the base of the loaded object to be static.

                              - **self_collision**: Sets the `URDF_USE_SELF_COLLISION` flag to allow self collisions.

                              - **globalScaling**: globalScaling will apply a scale factor to the URDF model.

                              - **flags**: Flags (see link below) that can be combined using a bitwise OR, |.

        :param node_params: A list containing the config of every :class:`~eagerx.core.entities.EngineNode` that represents
                            an :class:`~eagerx.core.entities.Object`'s sensor or actuator that is to be added.
        :param state_params: A list containing the parameters of every the :class:`~eagerx.core.entities.Object`'s
                             :class:`~eagerx.core.entities.EngineState` that is to be added.
        """
        obj_name = config["name"]
        entity_id = config["entity_id"]

        # add objects to simulator (we have a ref to the simulator with self.simulator)
        rospy.loginfo(f'Adding object "{obj_name}" of type "{entity_id}" to the simulator.')

        # Add self collision to flag
        if bridge_config["self_collision"]:
            flags = bridge_config["flags"] | pybullet.URDF_USE_SELF_COLLISION
        else:
            flags = bridge_config["flags"]

        # Add object
        if bridge_config["urdf"]:
            self.simulator["robots"][obj_name] = URDFBasedRobot(
                self._p,
                model_urdf=bridge_config["urdf"],  # Can be path (ending with .urdf), or ros param key to urdf (xml)string.
                robot_name=obj_name,
                basePosition=bridge_config["basePosition"],
                baseOrientation=bridge_config["baseOrientation"],
                fixed_base=bridge_config["fixed_base"],
                flags=flags,
            )
        else:  # if no urdf is provided, create dummy robot.
            self.simulator["robots"][obj_name] = None

    def pre_reset(self, *args, **kwargs):
        pass

    @register.states(erp=Float32, contactERP=Float32, frictionERP=Float32)
    def reset(self, erp: Float32 = None, contactERP: Float32 = None, frictionERP: Float32 = None):
        """Set any of the physics engine parameters (registered as states) if they were selected."""
        physics_engine_params = {}
        if erp:
            physics_engine_params["erp"] = erp.data
        if contactERP:
            physics_engine_params["contactERP"] = contactERP.data
        if frictionERP:
            physics_engine_params["frictionERP"] = frictionERP.data
        if len(physics_engine_params) > 0:
            self._p.setPhysicsEngineParameter(**physics_engine_params)

    @register.outputs(tick=UInt64)
    def callback(self, t_n: float, **kwargs: Dict[str, Union[List[Message], float, int]]):
        """Here, we step the world by 1/rate seconds."""
        self.simulator["world"].step()

    def shutdown(self) -> None:
        """Disconnects the bridge from the pybullet physics server"""
        self._p.disconnect()
