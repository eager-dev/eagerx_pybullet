import uuid
import numpy as np
import os

try:
    import pybullet
except ImportError as e:
    from gym import error

    raise error.DependencyNotInstalled("{}. (HINT: you need to install pybullet)".format(e))


class XmlBasedRobot:
    """
    Base class for mujoco .xml based agents.
    """

    self_collision = True

    def __init__(self, robot_name, self_collision):
        self.action_init = None
        self.parts = None
        self.objects = []
        self.jdict = None
        self.ordered_joints = None
        self.robot_body = None
        self.robot_name = robot_name
        self.robot_objectid = None
        self.self_collision = self_collision

    def addToScene(self, bullet_client, bodies):
        self._p = bullet_client

        if self.parts is not None:
            parts = self.parts
        else:
            parts = {}

        if self.jdict is not None:
            joints = self.jdict
        else:
            joints = {}

        if self.ordered_joints is not None:
            ordered_joints = self.ordered_joints
        else:
            ordered_joints = []

        if np.isscalar(bodies):  # streamline the case where bodies is actually just one body
            bodies = [bodies]

        dump = 0
        for i in range(len(bodies)):
            if self._p.getNumJoints(bodies[i]) == 0:
                part_name, robot_name = self._p.getBodyInfo(bodies[i])
                self.robot_name = robot_name.decode("utf8")
                part_name = part_name.decode("utf8")
                parts[part_name] = BodyPart(self._p, part_name, bodies, i, -1)
            for j in range(self._p.getNumJoints(bodies[i])):
                self._p.setJointMotorControl2(
                    bodies[i], j, pybullet.POSITION_CONTROL, positionGain=0.1, velocityGain=0.1, force=0
                )
                jointInfo = self._p.getJointInfo(bodies[i], j)
                joint_name = jointInfo[1]
                part_name = jointInfo[12]

                joint_name = joint_name.decode("utf8")
                part_name = part_name.decode("utf8")

                if dump:
                    print("ROBOT PART '%s'" % part_name)
                if dump:
                    print(
                        "ROBOT JOINT '%s'" % joint_name
                    )  # limits = %+0.2f..%+0.2f effort=%0.3f speed=%0.3f" % ((joint_name,) + j.limits()) )

                parts[part_name] = BodyPart(self._p, part_name, bodies, i, j)

                if part_name == self.robot_name:
                    self.robot_body = parts[part_name]

                if i == 0 and j == 0 and self.robot_body is None:  # if nothing else works, we take this as robot_body
                    parts[self.robot_name] = BodyPart(self._p, self.robot_name, bodies, 0, -1)
                    self.robot_body = parts[self.robot_name]

                if joint_name[:6] == "ignore":
                    Joint(self._p, joint_name, bodies, i, j).disable_motor()
                    continue

                if joint_name[:8] != "jointfix":
                    joints[joint_name] = Joint(self._p, joint_name, bodies, i, j)
                    ordered_joints.append(joints[joint_name])

                    joints[joint_name].power_coef = 100.0

                # TODO: Maybe we need this
                # joints[joint_name].power_coef, joints[joint_name].max_velocity = joints[joint_name].limits()[2:4]
                # self.ordered_joints.append(joints[joint_name])
                # self.jdict[joint_name] = joints[joint_name]

        return parts, joints, ordered_joints, self.robot_body

    # def reset_pose(self, position, orientation):
    #     self.parts[self.robot_name].reset_pose(position, orientation)


class URDFBasedRobot(XmlBasedRobot):
    """
    Base class for URDF .xml based robots.
    """

    def __init__(
        self,
        bullet_client,
        model_urdf,
        robot_name,
        basePosition=None,
        baseOrientation=None,
        fixed_base=False,
        self_collision=False,
        flags=0,
    ):
        XmlBasedRobot.__init__(self, robot_name, self_collision)

        self.model_urdf = model_urdf
        self.basePosition = basePosition if basePosition else [0, 0, 0]
        self.baseOrientation = baseOrientation if baseOrientation else [0, 0, 0, 1]
        self.fixed_base = fixed_base
        self.doneLoading = 0
        self.flags = flags
        self._load_robot(bullet_client)

    def _load_robot(self, bullet_client):
        self._p = bullet_client
        if self.doneLoading == 0:
            self.ordered_joints = []
            self.doneLoading = 1
            if self.model_urdf.endswith(".urdf"):  # Full path specified
                fullpath = self.model_urdf
            else:  # Write to /tmp file
                fullpath = f"/tmp/{uuid.uuid4().hex}.urdf"
                with open(fullpath, "w") as file:
                    file.write(self.model_urdf)
            if fullpath.startswith("/") and not os.path.exists(fullpath):
                raise IOError("File %s does not exist" % fullpath)
            # print(fullpath)

            if self.self_collision:
                flags = self.flags | pybullet.URDF_USE_SELF_COLLISION
            else:
                flags = self.flags
            self.robot_objectid = self._p.loadURDF(
                fullpath,
                basePosition=self.basePosition,
                baseOrientation=self.baseOrientation,
                useFixedBase=self.fixed_base,
                flags=flags,
            )
            if np.isscalar(self.robot_objectid):
                self.robot_objectid = [self.robot_objectid]
            self.parts, self.jdict, self.ordered_joints, self.robot_body = self.addToScene(self._p, self.robot_objectid)


class BodyPart:
    def __init__(self, bullet_client, body_name, bodies, bodyIndex, bodyPartIndex):
        # rospy.logdebug('bodyIndex %d | bodyPartIndex %d | name "%s"' % (bodyIndex, bodyPartIndex, body_name))
        self.bodies = bodies
        self._p = bullet_client
        self.bodyIndex = bodyIndex
        self.bodyPartIndex = bodyPartIndex
        self.initialPosition = self.current_position()
        self.initialOrientation = self.current_orientation()
        # self.bp_pose = Pose_Helper(self)

    def state_fields_of_pose_of(
        self, body_id, link_id=-1
    ):  # a method you will most probably need a lot to get pose and orientation
        if link_id == -1:
            (x, y, z), (a, b, c, d) = self._p.getBasePositionAndOrientation(body_id)
        else:
            (x, y, z), (a, b, c, d), _, _, _, _ = self._p.getLinkState(body_id, link_id)
        return np.array([x, y, z, a, b, c, d])

    def get_position(self):
        return self.current_position()

    def get_pose(self):
        return self.state_fields_of_pose_of(self.bodies[self.bodyIndex], self.bodyPartIndex)

    def speed(self):
        if self.bodyPartIndex == -1:
            (vx, vy, vz), (wx, wy, wz) = self._p.getBaseVelocity(self.bodies[self.bodyIndex])
        else:
            (x, y, z), (a, b, c, d), _, _, _, _, (vx, vy, vz), (vr, vp, vy) = self._p.getLinkState(
                self.bodies[self.bodyIndex], self.bodyPartIndex, computeLinkVelocity=1
            )
        return np.array([vx, vy, vz]), np.array([wx, wy, wz])

    def current_position(self):
        return self.get_pose()[:3]

    def current_orientation(self):
        return self.get_pose()[3:]

    def get_orientation(self):
        return self.current_orientation()

    # def reset_position(self, position):
    #     orientation = self._p.getBasePositionAndOrientation(self.bodyIndex)[1]
    #     self._p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, orientation)
    #
    # def reset_orientation(self, orientation):
    #     position = self._p.getBasePositionAndOrientation(self.bodyIndex)[0]
    #     self._p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, orientation)

    def reset_velocity(self, linearVelocity=None, angularVelocity=None):
        linearVelocity = linearVelocity if linearVelocity is not None else [0, 0, 0]
        angularVelocity = angularVelocity if angularVelocity is not None else [0, 0, 0]
        self._p.resetBaseVelocity(self.bodies[self.bodyIndex], linearVelocity, angularVelocity)

    def reset_pose(self, position, orientation):
        self._p.resetBasePositionAndOrientation(self.bodies[self.bodyIndex], position, orientation)

    # def pose(self):
    #     return self.bp_pose

    def contact_list(self):
        return self._p.getContactPoints(self.bodies[self.bodyIndex], -1, self.bodyPartIndex, -1)

    def get_bodyid_linkindex(self):
        return self.bodies[self.bodyIndex], self.bodyPartIndex

    def set_dynamic_parameter(self, parameter: str, value: float):
        d = {parameter: value}
        self._p.changeDynamics(self.bodies[self.bodyIndex], self.bodyPartIndex, **d)


class Joint:
    def __init__(self, bullet_client, joint_name, bodies, bodyIndex, jointIndex):
        # rospy.logdebug('bodyIndex %d | jointIndex %d | name "%s"' % (bodyIndex, jointIndex, joint_name))
        self.bodies = bodies
        self._p = bullet_client
        self.bodyIndex = bodyIndex
        self.jointIndex = jointIndex
        self.joint_name = joint_name

        jointInfo = self._p.getJointInfo(self.bodies[self.bodyIndex], self.jointIndex)
        self.lowerLimit = jointInfo[8]
        self.upperLimit = jointInfo[9]

        self.power_coeff = 0

    def set_state(self, x, vx):
        self._p.resetJointState(self.bodies[self.bodyIndex], self.jointIndex, x, vx)

    def current_position(self):  # just some synonyme method
        return self.get_state()

    def current_relative_position(self, get_absolute=False):
        pos, vel = self.get_state()
        pos_mid = 0.5 * (self.lowerLimit + self.upperLimit)
        if get_absolute:
            return (2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit), 0.1 * vel, pos, vel)
        else:
            return (2 * (pos - pos_mid) / (self.upperLimit - self.lowerLimit), 0.1 * vel)

    def get_state(self):
        x, vx, _, _ = self._p.getJointState(self.bodies[self.bodyIndex], self.jointIndex)
        return x, vx

    def get_position(self):
        x, _ = self.get_state()
        return x

    def get_orientation(self):
        _, r = self.get_state()
        return r

    def get_velocity(self):
        _, vx = self.get_state()
        return vx

    def get_joint_info(self):
        return self._p.getJointInfo(self.bodies[self.bodyIndex], self.jointIndex)

    def set_position(self, position):
        self._p.setJointMotorControl2(
            self.bodies[self.bodyIndex], self.jointIndex, pybullet.POSITION_CONTROL, targetPosition=position
        )

    def set_velocity(self, velocity):
        self._p.setJointMotorControl2(
            self.bodies[self.bodyIndex], self.jointIndex, pybullet.VELOCITY_CONTROL, targetVelocity=velocity
        )

    def set_motor_torque(self, torque):  # just some synonyme method
        self.set_torque(torque)

    def set_torque(self, torque):
        self._p.setJointMotorControl2(
            bodyIndex=self.bodies[self.bodyIndex],
            jointIndex=self.jointIndex,
            controlMode=pybullet.TORQUE_CONTROL,
            force=torque,
        )  # , positionGain=0.1, velocityGain=0.1)

    def reset_current_position(self, position, velocity):  # just some synonyme method
        self.reset_position(position, velocity)

    def reset_position(self, position, velocity):
        self._p.resetJointState(self.bodies[self.bodyIndex], self.jointIndex, targetValue=position, targetVelocity=velocity)
        self.disable_motor()

    def disable_motor(self):
        self._p.setJointMotorControl2(
            self.bodies[self.bodyIndex],
            self.jointIndex,
            controlMode=pybullet.POSITION_CONTROL,
            targetPosition=0,
            targetVelocity=0,
            positionGain=0.1,
            velocityGain=0.1,
            force=0,
        )

    def fix_joints(self):
        self._p.setJointMotorControl2(
            self.bodies[self.bodyIndex], self.jointIndex, controlMode=pybullet.VELOCITY_CONTROL, force=10**9
        )

    def get_bodyid_jointindex(self):
        return self.bodies[self.bodyIndex], self.jointIndex
