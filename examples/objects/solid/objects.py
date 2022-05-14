# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# EAGERx IMPORTS
from eagerx_pybullet.engine import PybulletEngine
from eagerx import Object, EngineNode, SpaceConverter, EngineState, Processor
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Solid(Object):
    entity_id = "Solid"

    @staticmethod
    @register.sensors(pos=Float32MultiArray, vel=Float32MultiArray, orientation=Float32MultiArray, angular_vel=Float32MultiArray)
    @register.engine_states(pos=Float32MultiArray, vel=Float32MultiArray, orientation=Float32MultiArray, angular_vel=Float32MultiArray)
    @register.config(urdf=None, fixed_base=True, self_collision=True, base_pos=[0, 0, 0], base_or=[0, 0, 0, 1])
    def agnostic(spec: ObjectSpec, rate):
        """This methods builds the agnostic definition for a solid object (e.g. can, table, etc...).

        Registered (agnostic) config parameters (should probably be set in the spec() function):
        - urdf: A fullpath (ending with .urdf), a key that points to the urdf (xml)string on the
                rosparam server, or a urdf within pybullet's search path. The `pybullet_data` package is
                included in the search path.
        - fixed_base: Force the base of the loaded object to be static.
        - self_collision: Enable self collisions.
        - base_pos: Base position of the object [x, y, z].
        - base_or: Base orientation of the object in quaternion [x, y, z, w].

        :param spec: Holds the desired configuration.
        :param rate: Rate (Hz) at which the callback is called.
        """
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Position
        spec.sensors.pos.rate = rate
        spec.sensors.pos.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-999, -999, -999],
            high=[999, 999, 999],
        )
        spec.states.pos.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-1, -1, 0],
            high=[1, 1, 0],
        )

        # Velocity
        spec.sensors.vel.rate = rate
        spec.sensors.vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-10, -10, -10],
            high=[10, 10, 10],
        )
        spec.states.vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0, 0, 0],
            high=[0, 0, 0],
        )

        # Orientation
        spec.sensors.orientation.rate = rate
        spec.sensors.orientation.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-1, -1, -1, -1],
            high=[1, 1, 1, 1],
        )
        spec.states.orientation.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0, 0, -1, -1],
            high=[0, 0, 1, 1],
        )

        # Angular velocity
        spec.sensors.angular_vel.rate = rate
        spec.sensors.angular_vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-10, -10, -10],
            high=[10, 10, 10],
        )
        spec.states.angular_vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0, 0, 0],
            high=[0, 0, 0],
        )

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
        spec: ObjectSpec,
        name: str,
        urdf: str,
        sensors=None,
        states=None,
        rate=30,
        base_pos=None,
        base_or=None,
        self_collision=True,
        fixed_base=True,
    ):
        """A spec to create a solid object (e.g. can, table, etc...).

        :param spec: The desired object configuration.
        :param name: Name of the object (topics are placed within this namespace).
        :param urdf: A fullpath (ending with .urdf), a key that points to the urdf (xml)string on the
                     rosparam server, or a urdf within pybullet's search path. The `pybullet_data` package is
                     included in the search path.
        :param sensors: A list of selected sensors. Must be a subset of the registered sensors.
        :param states: A list of selected states. Must be a subset of the registered actuators.
        :param rate: The default rate at which all sensors run. Can be modified via the spec API.
        :param base_pos: Base position of the object [x, y, z].
        :param base_or: Base orientation of the object in quaternion [x, y, z, w].
        :param self_collision: Enable self collisions.
        :param fixed_base: Force the base of the loaded object to be static.
        :return: ObjectSpec
        """
        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors is not None else ["pos", "vel", "orientation", "angular_vel"]
        spec.config.states = states if states is not None else ["pos", "vel", "orientation", "angular_vel"]

        # Add registered agnostic params
        spec.config.urdf = urdf
        spec.config.base_pos = base_pos if base_pos else [0, 0, 0]
        spec.config.base_or = base_or if base_or else [0, 0, 0, 1]
        spec.config.self_collision = self_collision
        spec.config.fixed_base = fixed_base

        # Add agnostic implementation
        Solid.agnostic(spec, rate)

    @staticmethod
    @register.engine(entity_id, PybulletEngine)
    def pybullet_engine(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this engine
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.engine_params(..) above the engine.add_object(...) method.
        spec.PybulletEngine.urdf = spec.config.urdf
        spec.PybulletEngine.basePosition = spec.config.base_pos
        spec.PybulletEngine.baseOrientation = spec.config.base_or
        spec.PybulletEngine.fixed_base = spec.config.fixed_base
        spec.PybulletEngine.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        spec.PybulletEngine.states.pos = EngineState.make("LinkState", mode="position")
        spec.PybulletEngine.states.vel = EngineState.make("LinkState", mode="velocity")
        spec.PybulletEngine.states.orientation = EngineState.make("LinkState", mode="orientation")
        spec.PybulletEngine.states.angular_vel = EngineState.make("LinkState", mode="angular_vel")

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        pos = EngineNode.make("LinkSensor", "pos", rate=spec.sensors.pos.rate, process=2, mode="position")
        vel = EngineNode.make("LinkSensor", "vel", rate=spec.sensors.vel.rate, process=2, mode="velocity")
        orientation = EngineNode.make("LinkSensor", "orientation", rate=spec.sensors.orientation.rate, process=2, mode="orientation")
        angular_vel = EngineNode.make("LinkSensor", "angular_vel", rate=spec.sensors.angular_vel.rate, process=2, mode="angular_vel")

        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        # Connect all engine nodes
        graph.add([pos, vel, orientation, angular_vel])
        graph.connect(source=pos.outputs.obs, sensor="pos")
        graph.connect(source=vel.outputs.obs, sensor="vel")
        graph.connect(source=orientation.outputs.obs, sensor="orientation")
        graph.connect(source=angular_vel.outputs.obs, sensor="angular_vel")

        # Check graph validity (commented out)
        # graph.is_valid(plot=True)
