import numpy as np
from eagerx_pybullet.engine import PybulletEngine
from eagerx import Object, Space
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Solid(Object):
    @classmethod
    @register.sensors(pos=None, vel=None,  orientation=None, angular_vel=None)
    @register.engine_states(pos=None, vel=None, orientation=None, angular_vel=None)
    def make(
        cls,
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
        """Make a spec to create a solid object (e.g. can, table, etc...).

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
        spec = cls.get_specification()

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

        # Position
        spec.sensors.pos.rate = rate
        spec.sensors.pos.space = Space(
            dtype="float32",
            low=np.array([-999, -999, -999], dtype="float32"),
            high=np.array([999, 999, 999], dtype="float32"),
        )
        spec.states.pos.space = Space(
            dtype="float32",
            low=np.array([-1, -1, 0], dtype="float32"),
            high=np.array([1, 1, 0], dtype="float32"),
        )

        # Velocity
        spec.sensors.vel.rate = rate
        spec.sensors.vel.space = Space(
            dtype="float32",
            low=np.array([-10, -10, -10], dtype="float32"),
            high=np.array([10, 10, 10], dtype="float32"),
        )
        spec.states.vel.space = Space(
            dtype="float32",
            low=np.array([0, 0, 0], dtype="float32"),
            high=np.array([0, 0, 0], dtype="float32"),
        )

        # Orientation
        spec.sensors.orientation.rate = rate
        spec.sensors.orientation.space = Space(
            dtype="float32",
            low=np.array([-1, -1, -1, -1], dtype="float32"),
            high=np.array([1, 1, 1, 1], dtype="float32"),
        )
        spec.states.orientation.space = Space(
            dtype="float32",
            low=np.array([0, 0, -1, -1], dtype="float32"),
            high=np.array([0, 0, 1, 1], dtype="float32"),
        )

        # Angular velocity
        spec.sensors.angular_vel.rate = rate
        spec.sensors.angular_vel.space = Space(
            dtype="float32",
            low=np.array([-10, -10, -10], dtype="float32"),
            high=np.array([10, 10, 10], dtype="float32"),
        )
        spec.states.angular_vel.space = Space(
            dtype="float32",
            low=np.array([0, 0, 0], dtype="float32"),
            high=np.array([0, 0, 0], dtype="float32"),
        )

        return spec

    @staticmethod
    @register.engine(PybulletEngine)
    def pybullet_engine(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this engine
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.engine_params(..) above the engine.add_object(...) method.
        spec.engine.urdf = spec.config.urdf
        spec.engine.basePosition = spec.config.base_pos
        spec.engine.baseOrientation = spec.config.base_or
        spec.engine.fixed_base = spec.config.fixed_base
        spec.engine.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        from eagerx_pybullet.enginestates import LinkState
        spec.engine.states.pos = LinkState.make(mode="position")
        spec.engine.states.vel = LinkState.make(mode="velocity")
        spec.engine.states.orientation = LinkState.make(mode="orientation")
        spec.engine.states.angular_vel = LinkState.make(mode="angular_vel")

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        from eagerx_pybullet.enginenodes import LinkSensor
        pos = LinkSensor.make("pos", rate=spec.sensors.pos.rate, process=2, mode="position")
        vel = LinkSensor.make("vel", rate=spec.sensors.vel.rate, process=2, mode="velocity")
        orientation = LinkSensor.make("orientation", rate=spec.sensors.orientation.rate, process=2, mode="orientation")
        angular_vel = LinkSensor.make("angular_vel", rate=spec.sensors.angular_vel.rate, process=2, mode="angular_vel")

        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        # Connect all engine nodes
        graph.add([pos, vel, orientation, angular_vel])
        graph.connect(source=pos.outputs.obs, sensor="pos")
        graph.connect(source=vel.outputs.obs, sensor="vel")
        graph.connect(source=orientation.outputs.obs, sensor="orientation")
        graph.connect(source=angular_vel.outputs.obs, sensor="angular_vel")
