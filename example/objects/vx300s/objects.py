import numpy as np
from typing import Optional, List
import os

# EAGERx IMPORTS
from eagerx_pybullet.engine import PybulletEngine
from eagerx import Object, Space
from eagerx.core.specs import ObjectSpec
from eagerx.core.graph_engine import EngineGraph
import eagerx.core.register as register


class Vx300s(Object):
    @classmethod
    @register.sensors(pos=None, vel=None, ft=None, at=None)
    @register.actuators(joint_control=None, gripper_control=None)
    @register.engine_states(pos=None, vel=None, gripper=None)
    def make(
        cls,
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
        """Make a spec to create a vx300s robot.

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
        spec = cls.get_specification()

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

        # Set observation properties: (space_converters, rate, etc...)
        spec.sensors.pos.rate = rate
        spec.sensors.pos.space = Space(
            dtype="float32",
            low=np.array([-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159], dtype="float32"),
            high=np.array([3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159], dtype="float32"),
        )
        spec.sensors.vel.rate = rate
        spec.sensors.vel.space = Space(
            dtype="float32",
            low=np.array([-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159], dtype="float32"),
            high=np.array([3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159], dtype="float32"),
        )
        spec.sensors.ft.rate = rate
        spec.sensors.ft.space = Space(
            dtype="float32",
            low=np.array(6*6*[-3.14159], dtype="float32"),   # 6 joints, each producing (Fx, Fy, Fz, Mx, My, Mz) measurements.
            high=np.array(6*6*[3.14159], dtype="float32"),  # 6 joints, each producing (Fx, Fy, Fz, Mx, My, Mz) measurements.
        )
        spec.sensors.at.rate = rate
        spec.sensors.at.space = Space(
            dtype="float32",
            low=np.array([-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159], dtype="float32"),
            high=np.array([3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159], dtype="float32"),
        )

        # Set actuator properties: (space_converters, rate, etc...)
        spec.actuators.joint_control.rate = rate
        spec.actuators.gripper_control.rate = rate
        spec.actuators.joint_control.space = Space(
            dtype="float32",
            low=np.array([-3.14159, -3.14159, -3.14159, -3.14159, -3.14159, -3.14159], dtype="float32"),
            high=np.array([3.14159, 3.14159, 3.14159, 3.14159, 3.14159, 3.14159], dtype="float32"),
        )
        from example.objects.vx300s.processors import MirrorAction
        spec.actuators.gripper_control.processor = MirrorAction.make(index=0, offset=0)
        spec.actuators.gripper_control.space = Space(
            dtype="float32",
            low=np.array([0.021, -0.057], dtype="float32"),
            high=np.array([0.057, -0.021], dtype="float32"),
        )

        # Set model_state properties: (space_converters)
        spec.states.pos.space = Space(
            dtype="float32",
            low=np.array([-3.14158, -1.85004, -1.76278, -3.14158, -1.86750, -3.14158], dtype="float32"),
            high=np.array([3.14158, 1.25663, 1.605702, 3.14158, 2.23402, 3.14158], dtype="float32"),
        )
        spec.states.vel.space = Space(
            dtype="float32",
            low=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype="float32"),
            high=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype="float32"),
        )
        spec.states.gripper.space = Space(
            dtype="float32",
            low=np.array([0.021, -0.057], dtype="float32"),
            high=np.array([0.057, -0.021], dtype="float32"),
        )

        return spec

    @staticmethod
    @register.engine(PybulletEngine)
    def pybullet_engine(spec: ObjectSpec, graph: EngineGraph):
        """Engine-specific implementation (Pybullet) of the object."""
        # Import any object specific entities for this engine
        import example.objects  # noqa # pylint: disable=unused-import
        import eagerx_pybullet  # noqa # pylint: disable=unused-import

        # Set object arguments (as registered per register.engine_params(..) above the engine.add_object(...) method.
        path = os.path.dirname(example.objects.__file__) + "/vx300s/descriptions/urdf/vx300s.urdf"
        spec.engine.urdf = path
        spec.engine.basePosition = spec.config.base_pos
        spec.engine.baseOrientation = spec.config.base_or
        spec.engine.fixed_base = spec.config.fixed_base
        spec.engine.self_collision = spec.config.self_collision

        # Create engine_states (no agnostic states defined in this case)
        from eagerx_pybullet.enginestates import JointState
        spec.engine.states.pos = JointState.make(joints=spec.config.joint_names, mode="position")
        spec.engine.states.vel = JointState.make(joints=spec.config.joint_names, mode="velocity")
        spec.engine.states.gripper = JointState.make(joints=spec.config.gripper_names, mode="position")

        # Create sensor engine nodes
        # Rate=None, but we will connect them to sensors (thus will use the rate set in the agnostic specification)
        from eagerx_pybullet.enginenodes import JointSensor, JointController
        pos_sensor = JointSensor.make(
            "pos_sensor", rate=spec.sensors.pos.rate, process=2, joints=spec.config.joint_names, mode="position"
        )
        vel_sensor = JointSensor.make(
            "vel_sensor", rate=spec.sensors.vel.rate, process=2, joints=spec.config.joint_names, mode="velocity"
        )
        ft_sensor = JointSensor.make(
            "ft_sensor", rate=spec.sensors.ft.rate, process=2, joints=spec.config.joint_names, mode="force_torque"
        )
        at_sensor = JointSensor.make(
            "at_sensor", rate=spec.sensors.at.rate, process=2, joints=spec.config.joint_names, mode="applied_torque"
        )

        # Create actuator engine nodes
        # Rate=None, but we will connect it to an actuator (thus will use the rate set in the agnostic specification)
        joint_control = JointController.make(
            "joint_control",
            rate=spec.actuators.joint_control.rate,
            process=2,
            joints=spec.config.joint_names,
            mode=spec.config.control_mode,
            vel_target=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            pos_gain=[0.45, 0.45, 0.65, 0.6, 0.45, 0.4],
            vel_gain=[1.7, 1.7, 1.5, 1.3, 1.0, 1.0],
        )
        gripper = JointController.make(
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
