from typing import Optional, List
import pybullet

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray

# IMPORT EAGERX
from eagerx.core.constants import process
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode
import eagerx.core.register as register


class JointSensor(EngineNode):
    @staticmethod
    @register.spec("JointSensor", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        joints: List[str],
        process: Optional[int] = process.BRIDGE,
        color: Optional[str] = "cyan",
        mode: str = "position",
    ):
        """JointSensor spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(JointSensor)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["obs"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.joints = joints
        spec.config.mode = mode

    def initialize(self, joints, mode):
        self.obj_name = self.config["name"]
        assert self.process == process.BRIDGE, (
            "Simulation node requires a reference to the simulator," " hence it must be launched in the Bridge process"
        )
        flag = self.obj_name in self.simulator["robots"]
        assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation node.'
        self.joints = joints
        self.mode = mode
        self.robot = self.simulator["robots"][self.obj_name]
        self._p = self.simulator["client"]
        self.physics_client_id = self._p._client

        self.bodyUniqueId = []
        self.jointIndices = []
        for _idx, pb_name in enumerate(joints):
            bodyid, jointindex = self.robot.jdict[pb_name].get_bodyid_jointindex()
            self.bodyUniqueId.append(bodyid), self.jointIndices.append(jointindex)
            if mode == "force_torque":
                self._p.enableJointForceTorqueSensor(
                    bodyUniqueId=bodyid, jointIndex=jointindex, enableSensor=True, physicsClientId=self.physics_client_id
                )
        self.joint_cb = self._joint_measurement(self._p, self.mode, self.bodyUniqueId[0], self.jointIndices)

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=UInt64)
    @register.outputs(obs=Float32MultiArray)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        obs = self.joint_cb()
        return dict(obs=Float32MultiArray(data=obs))

    @staticmethod
    def _joint_measurement(p, mode, bodyUniqueId, jointIndices):
        def cb():
            states = p.getJointStates(bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=p._client)
            obs = []
            if mode == "position":  # (x)
                for _i, (pos, _vel, _force_torque, _applied_torque) in enumerate(states):
                    obs.append(pos)
            elif mode == "velocity":  # (v)
                for _i, (_pos, vel, _force_torque, _applied_torque) in enumerate(states):
                    obs.append(vel)
            elif mode == "force_torque":  # (Fx, Fy, Fz, Mx, My, Mz)
                for _i, (_pos, _vel, force_torque, _applied_torque) in enumerate(states):
                    obs.append(force_torque)
            elif mode == "applied_torque":  # (T)
                for _i, (_pos, _vel, _force_torque, applied_torque) in enumerate(states):
                    obs.append(applied_torque)
            else:
                raise ValueError(f"Mode '{mode}' not recognized.")
            return obs

        return cb


class JointController(EngineNode):
    @staticmethod
    @register.spec("JointController", EngineNode)
    def spec(
        spec,
        name: str,
        rate: float,
        joints: List[str],
        process: Optional[int] = process.BRIDGE,
        color: Optional[str] = "green",
        mode: str = "position_control",
        vel_target: List[float] = None,
        pos_gain: List[float] = None,
        vel_gain: List[float] = None,
    ):
        """JointController spec"""
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(JointController)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick", "action"]
        spec.config.outputs = ["action_applied"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.joints = joints
        spec.config.mode = mode
        spec.config.vel_target = vel_target if vel_target else [0.0] * len(joints)
        spec.config.pos_gain = pos_gain if pos_gain else [0.2] * len(joints)
        spec.config.vel_gain = vel_gain if vel_gain else [0.2] * len(joints)

    def initialize(self, joints, mode, vel_target, pos_gain, vel_gain):
        # We will probably use self.simulator[self.obj_name] in callback & reset.
        self.obj_name = self.config["name"]
        assert self.process == process.BRIDGE, (
            "Simulation node requires a reference to the simulator," " hence it must be launched in the Bridge process"
        )
        flag = self.obj_name in self.simulator["robots"]
        assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation node.'
        self.joints = joints
        self.mode = mode
        self.vel_target = vel_target
        self.pos_gain = pos_gain
        self.vel_gain = vel_gain
        self.robot = self.simulator["robots"][self.obj_name]
        self._p = self.simulator["client"]
        self.physics_client_id = self._p._client

        self.bodyUniqueId = []
        self.jointIndices = []
        for _idx, pb_name in enumerate(joints):
            bodyid, jointindex = self.robot.jdict[pb_name].get_bodyid_jointindex()
            self.bodyUniqueId.append(bodyid), self.jointIndices.append(jointindex)
            if mode == "force_torque":
                self._p.enableJointForceTorqueSensor(
                    bodyUniqueId=bodyid, jointIndex=jointindex, enableSensor=True, physicsClientId=self.physics_client_id
                )

        self.joint_cb = self._joint_control(
            self._p, self.mode, self.bodyUniqueId[0], self.jointIndices, self.pos_gain, self.vel_gain, self.vel_target
        )

    @register.states()
    def reset(self):
        pass
        # self.simulator[self.obj_name]["input"] = np.squeeze(np.array(self.default_action))

    @register.inputs(tick=UInt64, action=Float32MultiArray)
    @register.outputs(action_applied=Float32MultiArray)
    def callback(
        self,
        t_n: float,
        tick: Optional[Msg] = None,
        action: Optional[Msg] = None,
    ):
        # Set action in pybullet
        self.joint_cb(action.msgs[-1].data)
        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])

    @staticmethod
    def _joint_control(p, mode, bodyUniqueId, jointIndices, pos_gain, vel_gain, vel_target):
        if mode == "position_control":

            def cb(action):
                return p.setJointMotorControlArray(
                    bodyUniqueId=bodyUniqueId,
                    jointIndices=jointIndices,
                    controlMode=pybullet.POSITION_CONTROL,
                    targetPositions=action,
                    targetVelocities=vel_target,
                    positionGains=pos_gain,
                    velocityGains=vel_gain,
                    physicsClientId=p._client,
                )

        elif mode == "velocity_control":

            def cb(action):
                return p.setJointMotorControlArray(
                    bodyUniqueId=bodyUniqueId,
                    jointIndices=jointIndices,
                    controlMode=pybullet.VELOCITY_CONTROL,
                    targetVelocities=action,
                    positionGains=pos_gain,
                    velocityGains=vel_gain,
                    physicsClientId=p._client,
                )

        elif mode == "pd_control":

            def cb(action):
                return p.setJointMotorControlArray(
                    bodyUniqueId=bodyUniqueId,
                    jointIndices=jointIndices,
                    controlMode=pybullet.PD_CONTROL,
                    targetVelocities=action,
                    positionGains=pos_gain,
                    velocityGains=vel_gain,
                    physicsClientId=p._client,
                )

        elif mode == "torque_control":

            def cb(action):
                return p.setJointMotorControlArray(
                    bodyUniqueId=bodyUniqueId,
                    jointIndices=jointIndices,
                    controlMode=pybullet.TORQUE_CONTROL,
                    forces=action,
                    positionGains=pos_gain,
                    velocityGains=vel_gain,
                    physicsClientId=p._client,
                )

        else:
            raise ValueError(f"Mode '{mode}' not recognized.")
        return cb
