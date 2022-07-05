from gym.spaces import Box
import numpy as np

from eagerx_pybullet.engine import PybulletEngine
from eagerx import Object
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Camera(Object):
    @classmethod
    @register.sensors(rgb=None,
                      rgba=None,
                      rgbd=None,
                      pos=None,
                      orientation=None)
    @register.engine_states(pos=None,
                            orientation=None)
    def make(
        cls,
        name: str,
        sensors=None,
        states=None,
        rate=30,
        base_pos=None,
        base_or=None,
        self_collision=True,
        fixed_base=True,
        render_shape=None,
        urdf: str = None,
        optical_link: str = None,
        calibration_link: str = None,
    ):
        """Make a spec to initialize a camera.

        :param name: Name of the object (topics are placed within this namespace).
        :param sensors: A list of selected sensors. Must be a subset of the registered sensors.
        :param states: A list of selected states. Must be a subset of the registered actuators.
        :param rate: The default rate at which all sensors run. Can be modified via the spec API.
        :param base_pos: Base position of the object [x, y, z].
        :param base_or: Base orientation of the object in quaternion [x, y, z, w].
        :param self_collision: Enable self collisions.
        :param fixed_base: Force the base of the loaded object to be static.
        :param render_shape: The shape of the produced images [height, width].
        :param urdf: A fullpath (ending with .urdf), a key that points to the urdf (xml)string on the
                     rosparam server, or a urdf within pybullet's search path. The `pybullet_data` package is
                     included in the search path.
        :param optical_link: Link related to the pose from which to render images.
        :param calibration_link: Link related to the pose that is reset.
        :return: ObjectSpec
        """
        spec = cls.get_specification()

        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if isinstance(sensors, list) else ["rgb"]
        spec.config.states = states if isinstance(states, list) else ["pos", "orientation"]

        # Add registered agnostic params
        spec.config.urdf = urdf
        spec.config.base_pos = base_pos if isinstance(base_pos, list) else [0, 0, 0]
        spec.config.base_or = base_or if isinstance(base_or, list) else [0, 0, 0, 1]
        spec.config.self_collision = self_collision
        spec.config.fixed_base = fixed_base
        spec.config.render_shape = render_shape if isinstance(render_shape, list) else [200, 300]
        spec.config.optical_link = optical_link if isinstance(optical_link, str) else None
        spec.config.calibration_link = calibration_link if isinstance(calibration_link, str) else None

        # Position
        spec.sensors.pos.rate = rate
        spec.sensors.pos.space = Box(
            dtype="float32",
            low=np.array([-999, -999, -999], dtype="float32"),
            high=np.array([999, 999, 999], dtype="float32"),
        )
        spec.states.pos.space = Box(
            dtype="float32",
            low=np.array([0.83, 0.0181, 0.75], dtype="float32"),
            high=np.array([0.83, 0.0181, 0.75], dtype="float32"),
        )

        # Orientation
        spec.sensors.orientation.rate = rate
        spec.sensors.orientation.space = Box(
            dtype="float32",
            low=np.array([-1, -1, -1, -1], dtype="float32"),
            high=np.array([1, 1, 1, 1], dtype="float32"),
        )
        spec.states.orientation.space = Box(
            dtype="float32",
            low=np.array([0.377, -0.04, -0.92, 0.088], dtype="float32"),
            high=np.array([0.377, -0.04, -0.92, 0.088], dtype="float32"),
        )

        # Rgb
        spec.sensors.rgb.rate = rate
        spec.sensors.rgb.space = Box(
            dtype="uint8",
            low=0,
            high=255,
            shape=tuple(spec.config.render_shape + [3]),
        )

        # Rgba
        spec.sensors.rgba.rate = rate
        spec.sensors.rgba.space = Box(
            dtype="uint8",
            low=0,
            high=255,
            shape=tuple(spec.config.render_shape + [4]),
        )

        # Rgbd
        spec.sensors.rgbd.rate = rate
        spec.sensors.rgbd.space = Box(
            dtype="uint8",
            low=0,
            high=255,
            shape=tuple(spec.config.render_shape + [4]),
        )

        return spec

    @staticmethod
    @register.engine(PybulletEngine)
    def pybullet_engine(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Set object arguments (as registered per register.engine_params(..) above the engine.add_object(...) method.
        import pybullet_data

        urdf = spec.config.urdf
        spec.engine.urdf = (
            urdf if isinstance(urdf, str) else "%s/%s.urdf" % (pybullet_data.getDataPath(), "cube_small")
        )
        # Initial position of baselink when urdf is loaded. Overwritten by state during the reset.
        spec.engine.basePosition = spec.config.base_pos
        # Initial orientation of baselink when urdf is loaded. Overwritten by state during the reset.
        spec.engine.baseOrientation = spec.config.base_or
        spec.engine.fixed_base = spec.config.fixed_base
        spec.engine.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        from eagerx_pybullet.enginestates import LinkState
        spec.engine.states.pos = LinkState.make(mode="position", link=spec.config.calibration_link)
        spec.engine.states.orientation = LinkState.make(mode="orientation", link=spec.config.calibration_link)

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        from eagerx_pybullet.enginenodes import LinkSensor, CameraSensor
        pos = LinkSensor.make("pos", rate=spec.sensors.pos.rate, process=2, mode="position", links=[spec.config.optical_link])
        orientation = LinkSensor.make(
            "orientation",
            rate=spec.sensors.orientation.rate,
            process=2,
            mode="orientation",
            links=[spec.config.optical_link],
        )

        # rgb
        rgb = CameraSensor.make("rgb", rate=spec.sensors.rgb.rate, process=2, mode="rgb", render_shape=spec.config.render_shape)
        rgb.config.inputs.append("pos")
        rgb.config.inputs.append("orientation")

        # rgba
        rgba = CameraSensor.make("rgba", rate=spec.sensors.rgb.rate, process=2, mode="rgba", render_shape=spec.config.render_shape)
        rgba.config.inputs.append("pos")
        rgba.config.inputs.append("orientation")

        # rgbd
        rgbd = CameraSensor.make("rgbd", rate=spec.sensors.rgb.rate, process=2, mode="rgbd", render_shape=spec.config.render_shape)
        rgbd.config.inputs.append("pos")
        rgbd.config.inputs.append("orientation")

        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        # Connect all engine nodes
        graph.add([pos, orientation, rgb, rgbd, rgba])
        graph.connect(source=pos.outputs.obs, sensor="pos")
        graph.connect(source=orientation.outputs.obs, sensor="orientation")
        graph.connect(source=rgb.outputs.image, sensor="rgb")
        graph.connect(source=rgba.outputs.image, sensor="rgba")
        graph.connect(source=rgbd.outputs.image, sensor="rgbd")
        graph.connect(source=pos.outputs.obs, target=rgb.inputs.pos)
        graph.connect(source=orientation.outputs.obs, target=rgb.inputs.orientation)
        graph.connect(source=pos.outputs.obs, target=rgba.inputs.pos)
        graph.connect(source=orientation.outputs.obs, target=rgba.inputs.orientation)
        graph.connect(source=pos.outputs.obs, target=rgbd.inputs.pos)
        graph.connect(source=orientation.outputs.obs, target=rgbd.inputs.orientation)
