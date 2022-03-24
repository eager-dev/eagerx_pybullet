from eagerx.core.entities import EngineState
import eagerx.core.register as register


class JointState(EngineState):
    @staticmethod
    @register.spec("JointState", EngineState)
    def spec(spec, joints, mode):
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
