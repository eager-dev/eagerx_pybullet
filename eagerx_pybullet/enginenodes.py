from typing import Optional, List, Dict
from scipy.spatial.transform import Rotation
import numpy as np
import pybullet

# IMPORT EAGERX
from eagerx import Space
from eagerx.core.specs import NodeSpec, ObjectSpec
from eagerx.core.constants import process as p
from eagerx.utils.utils import Msg
from eagerx.core.entities import EngineNode
import eagerx.core.register as register


class LinkSensor(EngineNode):
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        links: List[str] = None,
        process: Optional[int] = p.ENGINE,
        color: Optional[str] = "cyan",
        mode: str = "position",
    ):
        """A spec to create a LinkSensor node that provides sensor measurements for the specified set of links.

        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param links: List of links to be included in the measurements. Its order determines the ordering of the measurements.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.
        :param mode: Available: `position`, `orientation`, `velocity`, and `angular_vel`
        :return: NodeSpec
        """
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color)
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["obs"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.links = links if isinstance(links, list) else []
        spec.config.mode = mode
        return spec

    def initialize(self, spec: NodeSpec, object_spec: ObjectSpec, simulator: Dict):
        """Initializes the link sensor node according to the spec."""
        links = spec.config.links

        self.obj_name = object_spec.config.name
        assert self.process == p.ENGINE, (
            "Simulation node requires a reference to the simulator," " hence it must be launched in the Engine process"
        )
        flag = self.obj_name in simulator["robots"]
        assert flag, f'Simulator object "{simulator}" is not compatible with this simulation node.'
        self.robot = simulator["robots"][self.obj_name]
        # If no links are provided, take baselink
        if len(links) == 0:
            for pb_name, part in self.robot.parts.items():
                bodyid, linkindex = part.get_bodyid_linkindex()
                if linkindex == -1:
                    links.append(pb_name)
        self.links = links
        self.mode = spec.config.mode
        self._p = simulator["client"]
        self.physics_client_id = self._p._client
        self.link_cb = self._link_measurement(self._p, self.mode, self.robot, links)

    @register.states()
    def reset(self):
        """This link sensor is stateless, so nothing happens here."""
        pass

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(obs=Space(dtype="float32"))
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        """Produces a link sensor measurement called `obs`.

        The measurement is published at the specified rate * real_time_factor.

        Input `tick` ensures that this node is I/O synchronized with the simulator."""
        obs = self.link_cb()
        return dict(obs=np.array(obs, dtype="float32"))

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
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        joints: List[str],
        process: Optional[int] = p.ENGINE,
        color: Optional[str] = "cyan",
        mode: str = "position",
    ):
        """A spec to create a JointSensor node that provides sensor measurements for the specified set of joints.

        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param joints: List of joints to be included in the measurements. Its order determines the ordering of the
                       measurements.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.
        :param mode: Available: `position`, `velocity`, `force_torque`, and `applied_torque`.
        :return: NodeSpec
        """
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color)
        spec.config.inputs = ["tick"]
        spec.config.outputs = ["obs"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.joints = joints
        spec.config.mode = mode
        return spec

    def initialize(self, spec: NodeSpec, object_spec: ObjectSpec, simulator: Dict):
        """Initializes the joint sensor node according to the spec."""
        self.obj_name = object_spec.config.name
        assert self.process == p.ENGINE, (
            "Simulation node requires a reference to the simulator," " hence it must be launched in the Engine process"
        )
        flag = self.obj_name in simulator["robots"]
        assert flag, f'Simulator object "{simulator}" is not compatible with this simulation node.'
        self.joints = spec.config.joints
        self.mode = spec.config.mode
        self.robot = simulator["robots"][self.obj_name]
        self._p = simulator["client"]
        self.physics_client_id = self._p._client

        self.bodyUniqueId = []
        self.jointIndices = []
        for _idx, pb_name in enumerate(spec.config.joints):
            bodyid, jointindex = self.robot.jdict[pb_name].get_bodyid_jointindex()
            self.bodyUniqueId.append(bodyid), self.jointIndices.append(jointindex)
            if spec.config.mode == "force_torque":
                self._p.enableJointForceTorqueSensor(
                    bodyUniqueId=bodyid, jointIndex=jointindex, enableSensor=True, physicsClientId=self.physics_client_id
                )
        self.joint_cb = self._joint_measurement(self._p, self.mode, self.bodyUniqueId[0], self.jointIndices)

    @register.states()
    def reset(self):
        """This joint sensor is stateless, so nothing happens here."""
        pass

    @register.inputs(tick=Space(shape=(), dtype="int64"))
    @register.outputs(obs=Space(dtype="float32"))
    def callback(self, t_n: float, tick: Optional[Msg] = None):
        """Produces a joint sensor measurement called `obs`.

        The measurement is published at the specified rate * real_time_factor.

        Input `tick` ensures that this node is I/O synchronized with the simulator."""
        obs = self.joint_cb()
        return dict(obs=np.array(obs, dtype="float32"))

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
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        joints: List[str],
        process: Optional[int] = p.ENGINE,
        color: Optional[str] = "green",
        mode: str = "position_control",
        vel_target: List[float] = None,
        pos_gain: List[float] = None,
        vel_gain: List[float] = None,
        max_force: List[float] = None,
    ):
        """A spec to create a JointController node that controls a set of joints.

        For more info on `vel_target`, `pos_gain`, and `vel_gain`, see `setJointMotorControlMultiDofArray` in
        https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

        :param name: User specified node name.
        :param rate: Rate (Hz) at which the callback is called.
        :param joints: List of controlled joints. Its order determines the ordering of the applied commands.
        :param process: Process in which this node is launched. See :class:`~eagerx.core.constants.process` for all options.
        :param color: Specifies the color of logged messages & node color in the GUI.
        :param mode: Available: `position_control`, `velocity_control`, `pd_control`, and `torque_control`.
        :param vel_target: The desired velocity. Ordering according to `joints`.
        :param pos_gain: Position gain. Ordering according to `joints`.
        :param vel_gain: Velocity gain. Ordering according to `joints`.
        :param max_force: Maximum force when mode in [`position_control`, `velocity_control`, `pd_control`]. Ordering
                          according to `joints`.
        :return: NodeSpec
        """
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color)
        spec.config.inputs = ["tick", "action"]
        spec.config.outputs = ["action_applied"]

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.joints = joints
        spec.config.mode = mode
        spec.config.vel_target = vel_target if vel_target else [0.0] * len(joints)
        spec.config.pos_gain = pos_gain if pos_gain else [0.2] * len(joints)
        spec.config.vel_gain = vel_gain if vel_gain else [0.2] * len(joints)
        spec.config.max_force = max_force if max_force else [999.0] * len(joints)
        return spec

    def initialize(self, spec: NodeSpec, object_spec: ObjectSpec, simulator: Dict):
        """Initializes the joint controller node according to the spec."""
        # We will probably use simulator[self.obj_name] in callback & reset.
        self.obj_name = object_spec.config.name
        assert self.process == p.ENGINE, (
            "Simulation node requires a reference to the simulator," " hence it must be launched in the Engine process"
        )
        flag = self.obj_name in simulator["robots"]
        assert flag, f'Simulator object "{simulator}" is not compatible with this simulation node.'
        self.joints = spec.config.joints
        self.mode = spec.config.mode
        self.vel_target = spec.config.vel_target
        self.pos_gain = spec.config.pos_gain
        self.vel_gain = spec.config.vel_gain
        self.max_force = spec.config.max_force
        self.robot = simulator["robots"][self.obj_name]
        self._p = simulator["client"]
        self.physics_client_id = self._p._client

        self.bodyUniqueId = []
        self.jointIndices = []
        for _idx, pb_name in enumerate(spec.config.joints):
            bodyid, jointindex = self.robot.jdict[pb_name].get_bodyid_jointindex()
            self.bodyUniqueId.append(bodyid), self.jointIndices.append(jointindex)

        self.joint_cb = self._joint_control(
            self._p,
            self.mode,
            self.bodyUniqueId[0],
            self.jointIndices,
            self.pos_gain,
            self.vel_gain,
            self.vel_target,
            self.max_force,
        )

    @register.states()
    def reset(self):
        """This joint controller is stateless, so nothing happens here."""
        pass

    @register.inputs(tick=Space(shape=(), dtype="int64"), action=Space(dtype="float32"))
    @register.outputs(action_applied=Space(dtype="float32"))
    def callback(
        self,
        t_n: float,
        tick: Optional[Msg] = None,
        action: Optional[Msg] = None,
    ):
        """Sets the most recently received `action` in the pybullet joint controller.

        The action is set at the specified rate * real_time_factor.

        The output `action_applied` is the action that was set. If the input `action` comes in at a higher rate than
        this node's rate, `action_applied` may be differnt as only the most recently received `action` is set.

        Input `tick` ensures that this node is I/O synchronized with the simulator."""
        # Set action in pybullet
        self.joint_cb(action.msgs[-1])
        # Send action that has been applied.
        return dict(action_applied=action.msgs[-1])

    @staticmethod
    def _joint_control(p, mode, bodyUniqueId, jointIndices, pos_gain, vel_gain, vel_target, max_force):
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
                    forces=max_force,
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
                    forces=max_force,
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
                    forces=max_force,
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
    @classmethod
    def make(
        cls,
        name: str,
        rate: float,
        process: Optional[int] = p.ENGINE,
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
        spec = cls.get_specification()

        # Modify default node params
        spec.config.update(name=name, rate=rate, process=process, color=color)
        spec.config.inputs = inputs if isinstance(inputs, list) else ["tick"]
        spec.config.outputs = ["image"]

        # Add states if position and orientation are not inputs.
        spec.config.states = []
        if "pos" not in spec.config.inputs:
            spec.config.states.append("pos")
        if "orientation" not in spec.config.inputs:
            spec.config.states.append("orientation")

        # Set parameters, defined by the signature of cls.initialize(...)
        spec.config.update(mode=mode, fov=fov, near_val=near_val, far_val=far_val)
        spec.config.flags = pybullet.ER_NO_SEGMENTATION_MASK
        spec.config.renderer = pybullet.ER_BULLET_HARDWARE_OPENGL
        spec.config.render_shape = render_shape if isinstance(render_shape, list) else [480, 680]

        # Position
        spec.states.pos.space = Space(low=[-5, -5, 0], high=[5, 5, 5])

        # Orientation
        spec.states.orientation.space = Space(low=[-1, -1, -1, -1], high=[1, 1, 1, 1])

        # Image
        channels = 3 if mode == "rgb" else 4
        shape = (spec.config.render_shape[0], spec.config.render_shape[1], channels)
        spec.outputs.image.space = Space(low=0, high=255, shape=shape, dtype="uint8")
        return spec

    def initialize(self, spec: NodeSpec, object_spec: ObjectSpec, simulator: Dict):
        """Initializes the camera sensor according to the spec."""
        if simulator:
            self._p = simulator["client"]
        else:
            raise NotImplementedError("Currently, rendering leads to an error when connecting via shared memory.")
            # from pybullet_utils import bullet_client
            # self._p = bullet_client.BulletClient(pybullet.SHARED_MEMORY, options="-shared_memory_key 1234")
            # self._p = pybullet.connect(pybullet.SHARED_MEMORY, key=1234)
        # print("[rgb]: ", self._p._client)
        self.mode = spec.config.mode
        self.height, self.width = spec.config.render_shape
        self.intrinsic = dict(
            fov=spec.config.fov, nearVal=spec.config.near_val, farVal=spec.config.far_val, aspect=self.height / self.width
        )
        self.cb_args = dict(
            width=self.width,
            height=self.height,
            viewMatrix=None,
            projectionMatrix=None,
            flags=spec.config.flags,
            renderer=spec.config.renderer,
            physicsClientId=self._p._client,
        )
        self.cam_cb = self._camera_measurement(self._p, self.mode, self.cb_args)

    @register.states(pos=Space(dtype="float32"), orientation=Space(dtype="float32"))
    def reset(self, pos=None, orientation=None):
        """The static position and orientation of the camera sensor can be reset at the start of a new episode.

        If 'position' and 'orientation' were selected as inputs in the spec, nothing happens here because the camera pose
        changes over time according to the connected inputs.
        """
        self.cb_args["projectionMatrix"] = pybullet.computeProjectionMatrixFOV(**self.intrinsic)

        if pos is not None and orientation is not None:
            self.cb_args["viewMatrix"] = self._view_matrix(pos, orientation, self._p._client)
        if pos is not None:
            self.pos = pos
        if orientation is not None:
            self.orientation = orientation

    @register.inputs(tick=Space(shape=(), dtype="int64"), pos=Space(dtype="float32"), orientation=Space(dtype="float32"))
    @register.outputs(image=Space(dtype="uint8"))
    def callback(self, t_n: float, tick: Msg = None, pos: Msg = None, orientation: Msg = None):
        """Produces a camera sensor measurement called `image`.

        If 'position' and 'orientation' were selected as inputs in the spec, the pose of the camera is recalculated before
        rendering the image. Hence, this sensor is able to render images from the perspective of e.g. and end-effector.

        The image measurement is published at the specified rate * real_time_factor.

        Input `tick` ensures that this node is I/O synchronized with the simulator."""
        if pos:
            self.pos = pos.msgs[-1]
        if orientation:
            self.orientation = orientation.msgs[-1]

        if pos is not None or orientation is not None:
            self.cb_args["viewMatrix"] = self._view_matrix(self.pos, self.orientation, self._p._client)
        img = self.cam_cb()
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
