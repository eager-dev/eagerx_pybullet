from typing import Optional, List

# ROS IMPORTS
from std_msgs.msg import Float32MultiArray

# EAGERx IMPORTS
from eagerx_pybullet.engine import PybulletEngine
from eagerx import Object, EngineNode, SpaceConverter, EngineState, Processor
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register

# OTHER IMPORTS
import os


class Vx300s(Object):
    entity_id = "Vx300s"

    @staticmethod
    @register.sensors(pos=Float32MultiArray, vel=Float32MultiArray, ft=Float32MultiArray, at=Float32MultiArray)
    @register.actuators(joint_control=Float32MultiArray, gripper_control=Float32MultiArray)
    @register.engine_states(pos=Float32MultiArray, vel=Float32MultiArray, gripper=Float32MultiArray)
    @register.config(
        joint_names=None,
        gripper_names=None,
        fixed_base=True,
        self_collision=True,
        base_pos=None,
        base_or=None,
        control_mode=None,
    )
    def agnostic(spec: ObjectSpec, rate):
        """This methods builds the agnostic definition for a vx300s manipulator.

        Registered (agnostic) config parameters (should probably be set in the spec() function):
        - joint_names: List of arm joints.
        - gripper_names: List of gripper joints.
        - fixed_base: Force the base of the loaded object to be static.
        - self_collision: Enable self collisions.
        - base_pos: Base position of the object [x, y, z].
        - base_or: Base orientation of the object in quaternion [x, y, z, w].
        - control_mode: Control mode for the arm joints.
                        Available: `position_control`, `velocity_control`, `pd_control`, and `torque_control`.

        :param spec: Holds the desired configuration.
        :param rate: Rate (Hz) at which the callback is called.
        """
        # Register standard converters, space_converters, and processors
        import eagerx.converters  # noqa # pylint: disable=unused-import

        # Set observation properties: (space_converters, rate, etc...)
        spec.sensors.pos.rate = rate
        spec.sensors.pos.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
            high=[3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        spec.sensors.vel.rate = rate
        spec.sensors.vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
            high=[3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        spec.sensors.ft.rate = rate
        spec.sensors.ft.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=6*6*[-3.14159],   # 6 joints, each producing (Fx, Fy, Fz, Mx, My, Mz) measurements.
            high=6*6*[-3.14159],  # 6 joints, each producing (Fx, Fy, Fz, Mx, My, Mz) measurements.
        )
        spec.sensors.at.rate = rate
        spec.sensors.at.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
            high=[3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )

        # Set actuator properties: (space_converters, rate, etc...)
        spec.actuators.joint_control.rate = rate
        spec.actuators.gripper_control.rate = rate
        spec.actuators.joint_control.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159],
            high=[3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159],
        )
        spec.actuators.gripper_control.converter = Processor.make("MirrorAction", index=0, offset=0)
        spec.actuators.gripper_control.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", dtype="float32", low=[0.021], high=[0.057]
        )

        # Set model_state properties: (space_converters)
        spec.states.pos.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-3.14158, -1.85004, -1.76278, -3.14158, -1.86750, -3.14158],
            high=[3.14158, 1.25663, 1.605702, 3.14158, 2.23402, 3.14158],
        )
        spec.states.vel.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", dtype="float32", low=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], high=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        spec.states.gripper.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray", dtype="float32", low=[0.021, -0.057], high=[0.057, -0.021]
        )

    @staticmethod
    @register.spec(entity_id, Object)
    def spec(
        spec: ObjectSpec,
        name: str,
        sensors: List[str] = None,
        actuators: List[str] = None,
        states: List[str] = None,
        rate: float = 30.,
        base_pos: Optional[List[int]] = None,
        base_or: Optional[List[int]] = None,
        self_collision: bool = False,
        fixed_base: bool = True,
        control_mode: str = "position_control",
    ):
        """A spec to create a vx300s robot.

        :param spec: The desired object configuration.
        :param name: Name of the object (topics are placed within this namespace).
        :param sensors: A list of selected sensors. Must be a subset of the registered sensors.
        :param actuators: A list of selected actuators. Must be a subset of the registered actuators.
        :param states: A list of selected states. Must be a subset of the registered actuators.
        :param rate: The default rate at which all sensors and actuators run. Can be modified via the spec API.
        :param base_pos: Base position of the object [x, y, z].
        :param base_or: Base orientation of the object in quaternion [x, y, z, w].
        :param self_collision: Enable self collisions.
        :param fixed_base: Force the base of the loaded object to be static.
        :param control_mode: Control mode for the arm joints. Available: `position_control`, `velocity_control`, `pd_control`, and `torque_control`.
        :return: ObjectSpec
        """
        # Modify default agnostic params
        # Only allow changes to the agnostic params (rates, windows, (space)converters, etc...
        spec.config.name = name
        spec.config.sensors = sensors if sensors else ["pos", "vel", "ft", "at"]
        spec.config.actuators = actuators if actuators else ["joint_control", "gripper_control"]
        spec.config.states = states if states else ["pos", "vel", "gripper"]

        # Add registered agnostic params
        spec.config.joint_names = ["waist", "shoulder", "elbow", "forearm_roll", "wrist_angle", "wrist_rotate"]
        spec.config.gripper_names = ["left_finger", "right_finger"]
        spec.config.base_pos = base_pos if base_pos else [0, 0, 0]
        spec.config.base_or = base_or if base_or else [0, 0, 0, 1]
        spec.config.self_collision = self_collision
        spec.config.fixed_base = fixed_base
        spec.config.control_mode = control_mode

        # Add agnostic implementation
        Vx300s.agnostic(spec, rate)

    @staticmethod
    @register.engine(
        entity_id, PybulletEngine
    )  # This decorator pre-initializes engine implementation with default object_params
    def pybullet_engine(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this engine
        import examples.objects  # noqa # pylint: disable=unused-import
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.engine_params(..) above the engine.add_object(...) method.
        path = os.path.dirname(examples.objects.__file__) + "/vx300s/descriptions/urdf/vx300s.urdf"
        spec.PybulletEngine.urdf = path
        spec.PybulletEngine.basePosition = spec.config.base_pos
        spec.PybulletEngine.baseOrientation = spec.config.base_or
        spec.PybulletEngine.fixed_base = spec.config.fixed_base
        spec.PybulletEngine.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        spec.PybulletEngine.states.pos = EngineState.make("JointState", joints=spec.config.joint_names, mode="position")
        spec.PybulletEngine.states.vel = EngineState.make("JointState", joints=spec.config.joint_names, mode="velocity")
        spec.PybulletEngine.states.gripper = EngineState.make("JointState", joints=spec.config.gripper_names, mode="position")

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        pos_sensor = EngineNode.make(
            "JointSensor", "pos_sensor", rate=spec.sensors.pos.rate, process=2, joints=spec.config.joint_names, mode="position"
        )
        vel_sensor = EngineNode.make(
            "JointSensor", "vel_sensor", rate=spec.sensors.vel.rate, process=2, joints=spec.config.joint_names, mode="velocity"
        )
        ft_sensor = EngineNode.make(
            "JointSensor", "ft_sensor", rate=spec.sensors.ft.rate, process=2, joints=spec.config.joint_names, mode="force_torque"
        )
        at_sensor = EngineNode.make(
            "JointSensor", "at_sensor", rate=spec.sensors.at.rate, process=2, joints=spec.config.joint_names, mode="applied_torque"
        )

        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        joint_control = EngineNode.make(
            "JointController",
            "joint_control",
            rate=spec.actuators.joint_control.rate,
            process=2,
            joints=spec.config.joint_names,
            mode=spec.config.control_mode,
            vel_target=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            pos_gain=[0.45, 0.45, 0.65, 0.6, 0.45, 0.4],
            vel_gain=[1.7, 1.7, 1.5, 1.3, 1.0, 1.0],
        )
        gripper = EngineNode.make(
            "JointController",
            "gripper_control",
            rate=spec.actuators.gripper_control.rate,
            process=2,
            joints=spec.config.gripper_names,
            mode="position_control",
            vel_target=[0.0, 0.0],
            pos_gain=[1.5, 1.5],
            vel_gain=[0.7, 0.7],
        )

        # Connect all engine nodes
        graph.add([pos_sensor, joint_control, gripper, vel_sensor, ft_sensor, at_sensor])
        graph.connect(source=pos_sensor.outputs.obs, sensor="pos")
        graph.connect(source=vel_sensor.outputs.obs, sensor="vel")
        graph.connect(source=ft_sensor.outputs.obs, sensor="ft")
        graph.connect(source=at_sensor.outputs.obs, sensor="at")
        graph.connect(actuator="joint_control", target=joint_control.inputs.action)
        graph.connect(actuator="gripper_control", target=gripper.inputs.action)

        # Check graph validity (commented out)
        # graph.is_valid(plot=True)
