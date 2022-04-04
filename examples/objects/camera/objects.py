from scipy.spatial.transform import Rotation

# ROS IMPORTS
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image

# EAGERx IMPORTS
from eagerx_pybullet.bridge import PybulletBridge
from eagerx import Object, EngineNode, SpaceConverter, EngineState
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Camera(Object):
    entity_id = "Camera"

    @staticmethod
    @register.sensors(rgb=Image, rgba=Image, rgbd=Image, pos=Float32MultiArray, orientation=Float32MultiArray)
    @register.engine_states(pos=Float32MultiArray, orientation=Float32MultiArray)
    @register.config(
        urdf=None,
        fixed_base=True,
        self_collision=True,
        base_pos=[0, 0, 0],
        base_or=[0, 0, 0, 1],
        render_shape=[480, 640],
        optical_link=None,
        calibration_link=None,
    )
    def agnostic(spec: ObjectSpec, rate):
        """This methods builds the agnostic definition for a camera.

        Registered (agnostic) config parameters (should probably be set in the spec() function):
        - urdf: A fullpath (ending with .urdf), a key that points to the urdf (xml)string on the
                rosparam server, or a urdf within pybullet's search path. The `pybullet_data` package is
                included in the search path.
        - fixed_base: Force the base of the loaded object to be static.
        - self_collision: Enable self collisions.
        - base_pos: Base position of the object [x, y, z].
        - base_or: Base orientation of the object in quaternion [x, y, z, w].
        - render_shape: The shape of the produced images [height, width].
        - optical_link: Link related to the pose from which to render images.
        - calibration_link: Link related to the pose that is reset.

        :param spec: Holds the desired configuration.
        :param rate: Rate (Hz) at which the callback is called.
        """

        # """Agnostic definition of the Camera"""
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
            low=[0.83, 0.0181, 0.75],
            high=[0.83, 0.0181, 0.75],
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
            low=[0.377, -0.04, -0.92, 0.088],
            high=[0.377, -0.04, -0.92, 0.088],
        )

        # Rgb
        spec.sensors.rgb.rate = rate
        spec.sensors.rgb.space_converter = SpaceConverter.make(
            "Space_Image",
            dtype="float32",
            low=0,
            high=1,
            shape=spec.config.render_shape + [3],
        )

        # Rgba
        spec.sensors.rgba.rate = rate
        spec.sensors.rgba.space_converter = SpaceConverter.make(
            "Space_Image",
            dtype="float32",
            low=0,
            high=1,
            shape=spec.config.render_shape + [4],
        )

        # Rgbd
        spec.sensors.rgbd.rate = rate
        spec.sensors.rgbd.space_converter = SpaceConverter.make(
            "Space_Image",
            dtype="float32",
            low=0,
            high=1,
            shape=spec.config.render_shape + [4],
        )

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
        spec: ObjectSpec,
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
        """A spec to create a camera.

        :param spec: The desired object configuration.
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
        # Performs all the steps to fill-in the params with registered info about all functions.
        Camera.initialize_spec(spec)

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

        # Add agnostic implementation
        Camera.agnostic(spec, rate)

    @staticmethod
    @register.bridge(entity_id, PybulletBridge)
    def pybullet_bridge(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this bridge
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.bridge_params(..) above the bridge.add_object(...) method.
        import pybullet_data

        urdf = spec.config.urdf
        spec.PybulletBridge.urdf = (
            urdf if isinstance(urdf, str) else "%s/%s.urdf" % (pybullet_data.getDataPath(), "cube_small")
        )
        # Initial position of baselink when urdf is loaded. Overwritten by state during the reset.
        spec.PybulletBridge.basePosition = spec.config.base_pos
        # Initial orientation of baselink when urdf is loaded. Overwritten by state during the reset.
        spec.PybulletBridge.baseOrientation = spec.config.base_or
        spec.PybulletBridge.fixed_base = spec.config.fixed_base
        spec.PybulletBridge.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        spec.PybulletBridge.states.pos = EngineState.make("BaseState", mode="position", link=spec.config.calibration_link)
        spec.PybulletBridge.states.orientation = EngineState.make(
            "BaseState", mode="orientation", link=spec.config.calibration_link
        )

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        pos = EngineNode.make(
            "LinkSensor", "pos", rate=spec.sensors.pos.rate, process=2, mode="position", links=[spec.config.optical_link]
        )
        orientation = EngineNode.make(
            "LinkSensor",
            "orientation",
            rate=spec.sensors.orientation.rate,
            process=2,
            mode="orientation",
            links=[spec.config.optical_link],
        )

        # rgb
        rgb = EngineNode.make(
            "CameraSensor", "rgb", rate=spec.sensors.rgb.rate, process=2, mode="rgb", render_shape=spec.config.render_shape
        )
        rgb.config.inputs.append("pos")
        rgb.config.inputs.append("orientation")

        # rgba
        rgba = EngineNode.make(
            "CameraSensor", "rgba", rate=spec.sensors.rgb.rate, process=2, mode="rgba", render_shape=spec.config.render_shape
        )
        rgba.config.inputs.append("pos")
        rgba.config.inputs.append("orientation")

        # rgbd
        rgbd = EngineNode.make(
            "CameraSensor", "rgbd", rate=spec.sensors.rgb.rate, process=2, mode="rgbd", render_shape=spec.config.render_shape
        )
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

        # Check graph validity (commented out)
        # graph.is_valid(plot=True)
