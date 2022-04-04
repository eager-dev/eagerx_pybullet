from typing import Optional, List
from scipy.spatial.transform import Rotation
import pybullet

# IMPORT ROS
from std_msgs.msg import UInt64, Float32MultiArray
from sensor_msgs.msg import Image

# IMPORT EAGERX
from eagerx.core.specs import NodeSpec
from eagerx.core.constants import process as p
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode, SpaceConverter
import eagerx.core.register as register


class LinkSensor(EngineNode):
    @staticmethod
    @register.spec("LinkSensor", EngineNode)
    def spec(
        spec: NodeSpec,
        name: str,
        rate: float,
        links: List[str] = None,
        process: Optional[int] = p.BRIDGE,
        color: Optional[str] = "cyan",
        mode: str = "position",
    ):
        """A spec to create a LinkSensor node that provides sensor measurements for the specified set of links.

        :param spec: Holds the desired configuration.
        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param links: List of links to be included in the measurements. Its order determines the ordering of the measurements.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.
        :param mode: Available: `position`, `orientation`, `velocity`, and `angular_vel`
        :return: NodeSpec
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(LinkSensor)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["obs"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.links = links if isinstance(links, list) else []
        spec.config.mode = mode

    def initialize(self, links, mode):
        self.obj_name = self.config["name"]
        assert self.process == p.BRIDGE, (
            "Simulation node requires a reference to the simulator," " hence it must be launched in the Bridge process"
        )
        flag = self.obj_name in self.simulator["robots"]
        assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation node.'
        self.robot = self.simulator["robots"][self.obj_name]
        # If no links are provided, take baselink
        if len(links) == 0:
            for pb_name, part in self.robot.parts.items():
                bodyid, linkindex = part.get_bodyid_linkindex()
                if linkindex == -1:
                    links.append(pb_name)
        self.links = links
        self.mode = mode
        self._p = self.simulator["client"]
        self.physics_client_id = self._p._client
        self.link_cb = self._link_measurement(self._p, self.mode, self.robot, links)

    @register.states()
    def reset(self):
        pass

    @register.inputs(tick=UInt64)
    @register.outputs(obs=Float32MultiArray)
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        obs = self.link_cb()
        return dict(obs=Float32MultiArray(data=obs))

    @staticmethod
    def _link_measurement(p, mode, robot, links):
        if mode == "position":  # (x, y, z)

            def cb():
                obs = []
                for pb_name in links:
                    obs += robot.parts[pb_name].get_position().tolist()
                return obs

        elif mode == "orientation":  # (x, y, z, w)

            def cb():
                obs = []
                for pb_name in links:
                    obs += robot.parts[pb_name].get_orientation().tolist()
                return obs

        elif mode == "velocity":  # (vx, vy, vz)

            def cb():
                obs = []
                for pb_name in links:
                    vel, _ = robot.parts[pb_name].speed()
                    obs += vel.tolist()
                return obs

        elif mode == "angular_vel":  # (vx, vy, vz)

            def cb():
                obs = []
                for pb_name in links:
                    _, rate = robot.parts[pb_name].speed()
                    obs += rate.tolist()
                return obs

        else:
            raise ValueError(f"Mode '{mode}' not recognized.")
        return cb


class JointSensor(EngineNode):
    @staticmethod
    @register.spec("JointSensor", EngineNode)
    def spec(
        spec: NodeSpec,
        name: str,
        rate: float,
        joints: List[str],
        process: Optional[int] = p.BRIDGE,
        color: Optional[str] = "cyan",
        mode: str = "position",
    ):
        """A spec to create a JointSensor node that provides sensor measurements for the specified set of joints.

        :param spec: Holds the desired configuration in a Spec object.
        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param joints: List of joints to be included in the measurements. Its order determines the ordering of the
                       measurements.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.
        :param mode: Available: `position`, `velocity`, `force_torque`, and `applied_torque`.
        :return: NodeSpec
        """
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
        assert self.process == p.BRIDGE, (
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
                    obs += list(force_torque)
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
        spec: NodeSpec,
        name: str,
        rate: float,
        joints: List[str],
        process: Optional[int] = p.BRIDGE,
        color: Optional[str] = "green",
        mode: str = "position_control",
        vel_target: List[float] = None,
        pos_gain: List[float] = None,
        vel_gain: List[float] = None,
    ):
        """A spec to create a JointController node that controls a set of joints.

        For more info on `vel_target`, `pos_gain`, and `vel_gain`, see `setJointMotorControlMultiDofArray` in
        https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

        :param spec: Holds the desired configuration in a Spec object.
        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param joints: List of controlled joints. Its order determines the ordering of the applied commands.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.
        :param mode: Available: `position_control`, `velocity_control`, `pd_control`, and `torque_control`.
        :param vel_target: The desired velocity. Ordering according to `joints`.
        :param pos_gain: Position gain. Ordering according to `joints`.
        :param vel_gain: Velocity gain. Ordering according to `joints`.
        :return: NodeSpec
        """
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
        assert self.process == p.BRIDGE, (
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


class CameraSensor(EngineNode):
    @staticmethod
    @register.spec("CameraSensor", EngineNode)
    def spec(
        spec: NodeSpec,
        name: str,
        rate: float,
        process: Optional[int] = p.BRIDGE,
        color: Optional[str] = "cyan",
        mode: str = "rgb",
        inputs: List[str] = None,
        render_shape: List[int] = None,
        fov: float = 57.0,
        near_val: float = 0.1,
        far_val: float = 100.0,
    ):
        """A spec to create a CameraSensor node that provides images that can be used for perception and/or rendering.

        For more info on `fov`, `near_val`, and `far_val`, see the `Synthetic Camera Rendering` section in
        https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.33wr3gwy5kuj

        :param spec: Holds the desired configuration in a Spec object.
        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.
        :param mode: Available: `rgb`, `rgbd`, and `rgba`.
        :param inputs: Optionally, if the camera pose changes over time select `pos` and/or `orientation` & connect
                       accordingly, to dynamically change the camera view. If not selected, `pos` and/or `orientation` are
                       selected as states. This means you can choose a static camera pose at the start of every episode.
        :param render_shape: The shape of the produced images [height, width].
        :param fov: Field of view.
        :param near_val: Near plane distance.
        :param far_val: Far plane distance.
        :return: NodeSpec
        """
        # Performs all the steps to fill-in the params with registered info about all functions.
        spec.initialize(CameraSensor)

        # Modify default node params
        spec.config.name = name
        spec.config.rate = rate
        spec.config.process = process
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = ["image"]

        # Add states if position and orientation are not inputs.
        spec.config.states = []
        if "pos" not in spec.config.inputs:
            spec.config.states.append("pos")
        if "orientation" not in spec.config.inputs:
            spec.config.states.append("orientation")

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.mode = mode
        spec.config.render_shape = render_shape if isinstance(render_shape, list) else [480, 680]
        spec.config.fov = fov
        spec.config.near_val = near_val
        spec.config.far_val = far_val

        # Position
        spec.states.pos.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[-1, -1, 0],
            high=[1, 1, 0],
        )

        # Orientation
        spec.states.orientation.space_converter = SpaceConverter.make(
            "Space_Float32MultiArray",
            dtype="float32",
            low=[0, 0, -1, -1],
            high=[0, 0, 1, 1],
        )

    def initialize(
        self,
        mode,
        render_shape,
        fov=57,
        near_val=0.1,
        far_val=100,
        flags=pybullet.ER_NO_SEGMENTATION_MASK,
        renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
    ):
        if self.simulator:
            self._p = self.simulator["client"]
        else:
            raise NotImplementedError("Currently, rendering leads to an error when connecting via shared memory.")
            # from pybullet_utils import bullet_client
            # self._p = bullet_client.BulletClient(pybullet.SHARED_MEMORY, options="-shared_memory_key 1234")
            # self._p = pybullet.connect(pybullet.SHARED_MEMORY, key=1234)
        # print("[rgb]: ", self._p._client)
        self.mode = mode
        self.height, self.width = render_shape
        self.intrinsic = dict(fov=fov, nearVal=near_val, farVal=far_val, aspect=self.height / self.width)
        self.cb_args = dict(
            width=self.width,
            height=self.height,
            viewMatrix=None,
            projectionMatrix=None,
            flags=flags,
            renderer=renderer,
            physicsClientId=self._p._client,
        )
        self.cam_cb = self._camera_measurement(self._p, self.mode, self.cb_args)

    @register.states(pos=Float32MultiArray, orientation=Float32MultiArray)
    def reset(self, pos=None, orientation=None):
        self.cb_args["projectionMatrix"] = pybullet.computeProjectionMatrixFOV(**self.intrinsic)

        if pos is not None and orientation is not None:
            self.cb_args["viewMatrix"] = self._view_matrix(pos.data, orientation.data, self._p._client)
        if pos:
            self.pos = pos.data
        if orientation:
            self.orientation = orientation.data

    @register.inputs(tick=UInt64, pos=Float32MultiArray, orientation=Float32MultiArray)
    @register.outputs(image=Image)
    def callback(self, t_n: float, tick: Msg = None, pos: Msg = None, orientation: Msg = None):
        if pos:
            self.pos = pos.msgs[-1].data
        if orientation:
            self.orientation = orientation.msgs[-1].data

        if pos is not None or orientation is not None:
            self.cb_args["viewMatrix"] = self._view_matrix(self.pos, self.orientation, self._p._client)
        obs = self.cam_cb()
        img = Image(
            data=obs.tobytes("C"), height=self.height, width=self.width, encoding="rgb8", step=obs.shape[-1] * self.width
        )
        return dict(image=img)

    @staticmethod
    def _view_matrix(position, orientation, physicsClientId):
        r = Rotation.from_quat(orientation)
        cameraEyePosition = position
        cameraTargetPosition = r.as_matrix()[:, 2]
        cameraUpVector = -r.as_matrix()[:, 1]
        return pybullet.computeViewMatrix(
            cameraEyePosition, cameraTargetPosition, cameraUpVector, physicsClientId=physicsClientId
        )

    @staticmethod
    def _camera_measurement(p, mode, cb_args):
        if mode == "rgb":

            def cb():
                _, _, rgba, depth, seg = p.getCameraImage(**cb_args)
                return rgba[:, :, :3]

        elif mode == "rgba":

            def cb():
                _, _, rgba, depth, seg = p.getCameraImage(**cb_args)
                return rgba[:, :, :4]

        elif mode == "rgbd":

            def cb():
                _, _, rgba, depth, seg = p.getCameraImage(**cb_args)
                depth *= 255  # Convert depth to uint8
                rgba[:, :, 3] = depth.astype("uint8")  # replace alpha channel with depth
                return rgba[:, :, :4]

        else:
            raise ValueError(f"Mode '{mode}' not recognized.")
        return cb
