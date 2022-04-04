from typing import List, Optional
from eagerx.core.entities import EngineState
import eagerx.core.register as register
from eagerx.core.specs import EngineStateSpec
import numpy as np
from scipy.spatial.transform import Rotation


class JointState(EngineState):
    @staticmethod
    @register.spec("JointState", EngineState)
    def spec(spec: EngineStateSpec, joints: List[str], mode: str):
        """A spec to create an EngineState that resets a set of specified joints to the desired state.

        :param spec: Holds the desired configuration in a Spec object.
        :param joints: List of joints to be reset. Its order determines the ordering of the reset.
        :param mode: Available: `position`, `velocity`.
        :return: EngineStateSpec
        """
        spec.initialize(JointState)
        spec.config.joints = joints
        spec.config.mode = mode

    def initialize(self, joints, mode):
        self.obj_name = self.config["name"]
        flag = self.obj_name in self.simulator["robots"]
        assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation state.'
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
        self.joint_cb = self._joint_reset(self._p, self.mode, self.bodyUniqueId[0], self.jointIndices)

    def reset(self, state, done):
        if not done:
            self.joint_cb(state.data)

    @staticmethod
    def _joint_reset(p, mode, bodyUniqueId, jointIndices):
        if mode == "position":

            def cb(state):
                # Only 1-dof joints are supported here.
                # https://github.com/bulletphysics/bullet3/issues/2803
                velocities = []
                states = p.getJointStates(bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=p._client)
                for _i, (_, vel, _, _) in enumerate(states):
                    velocities.append([vel])
                p.resetJointStatesMultiDof(
                    targetValues=[[s] for s in state],
                    targetVelocities=velocities,
                    bodyUniqueId=bodyUniqueId,
                    jointIndices=jointIndices,
                    physicsClientId=p._client,
                )

        elif mode == "velocity":

            def cb(state):
                positions = []
                states = p.getJointStates(bodyUniqueId=bodyUniqueId, jointIndices=jointIndices, physicsClientId=p._client)
                for _i, (pos, _, _, _) in enumerate(states):
                    positions.append([pos])
                p.resetJointStatesMultiDof(
                    targetValues=positions,
                    targetVelocities=[[s] for s in state],
                    bodyUniqueId=bodyUniqueId,
                    jointIndices=jointIndices,
                    physicsClientId=p._client,
                )

        else:
            raise ValueError(f"Mode '{mode}' not recognized.")
        return cb


class BaseState(EngineState):
    @staticmethod
    @register.spec("BaseState", EngineState)
    def spec(spec: EngineStateSpec, mode: str, link: Optional[str] = None):
        """A spec to create an EngineState that resets a specified link to the desired state.

        :param spec: Holds the desired configuration in a Spec object.
        :param mode: Available: `position`, `orientation`, 'velocity', and 'angular_vel`.
        :param link: The link that is to be reset. By default, the baselink is reset.
        :return: EngineStateSpec
        """
        spec.initialize(BaseState)
        spec.config.mode = mode
        spec.config.link = link

    def initialize(self, mode, link=None):
        self.obj_name = self.config["name"]
        flag = self.obj_name in self.simulator["robots"]
        assert flag, f'Simulator object "{self.simulator}" is not compatible with this simulation state.'
        self.mode = mode
        self.robot = self.simulator["robots"][self.obj_name]
        if link is None:
            for _pb_name, part in self.robot.parts.items():
                bodyid, linkindex = part.get_bodyid_linkindex()
                if linkindex == -1:
                    self.bodypart = part
        else:
            self.bodypart = self.robot.parts[link]
        self._p = self.simulator["client"]
        self.physics_client_id = self._p._client
        self.bodyUniqueId = self.robot.robot_objectid
        self.base_cb = self._base_reset(self._p, self.mode, self.bodypart, self.bodyUniqueId[0])

    def reset(self, state, done):
        self.base_cb(state.data)

    @staticmethod
    def _base_reset(p, mode, bodypart, bodyUniqueId):
        if mode == "position":

            def cb(state):
                base_pose = p.getBasePositionAndOrientation(bodyUniqueId)
                # Correct state if not baselink
                if bodypart.bodyPartIndex > -1:
                    pose = bodypart.get_pose()
                    pos_offset = pose[:3] - np.array(base_pose[0])
                    state -= pos_offset
                bodypart.reset_pose(state, base_pose[1])

        elif mode == "orientation":

            def cb(state):
                # Correct state if not baselink
                if bodypart.bodyPartIndex > -1:
                    pose = bodypart.get_pose()
                    pos_desired = pose[:3]

                    base_pose = p.getBasePositionAndOrientation(bodyUniqueId)

                    # Rotation matrix
                    R_g_cam = Rotation.from_quat(pose[3:].tolist())
                    R_g_base = Rotation.from_quat(base_pose[1])
                    R_desired = Rotation.from_quat(state)

                    quat_base_desired = Rotation.from_matrix(
                        np.matmul(np.matmul(R_desired.as_matrix(), R_g_cam.as_matrix().transpose()), R_g_base.as_matrix())
                    ).as_quat()
                    bodypart.reset_pose(base_pose[0], quat_base_desired)

                    # Correct position after rotating
                    pos_offset = bodypart.get_pose()[:3] - np.array(p.getBasePositionAndOrientation(bodyUniqueId)[0])
                    base_pose = p.getBasePositionAndOrientation(bodyUniqueId)
                    bodypart.reset_pose(pos_desired - pos_offset, base_pose[1])
                else:
                    base_pose = p.getBasePositionAndOrientation(bodyUniqueId)
                    bodypart.reset_pose(base_pose[0], state)

        elif mode == "velocity":

            def cb(state):
                if bodypart.bodyPartIndex > -1:
                    raise NotImplementedError("This node does not support resetting links any other than the baselink.")
                _, angular_vel = p.getBaseVelocity(bodyUniqueId, physicsClientId=p._client)
                bodypart.reset_velocity(linearVelocity=state, angularVelocity=angular_vel)

        elif mode == "angular_vel":

            def cb(state):
                vel, _ = p.getBaseVelocity(bodyUniqueId, physicsClientId=p._client)
                bodypart.reset_velocity(linearVelocity=vel, angularVelocity=state)

        else:
            raise ValueError(f"Mode '{mode}' not recognized.")
        return cb
