import eagerx

# OTHER
import os
import pytest

NP = eagerx.process.NEW_PROCESS
ENV = eagerx.process.ENVIRONMENT


@pytest.mark.timeout(60)
@pytest.mark.parametrize("control_mode", ["position_control", "pd_control", "torque_control", "velocity_control"])
@pytest.mark.parametrize("p", [ENV, NP])
def test_eagerx_pybullet(control_mode, p):
    eagerx.set_log_level(eagerx.WARN)

    # Define unique name for test environment
    name = f"{control_mode}_{p}"
    engine_p = p
    rate = 30

    # Initialize empty graph
    graph = eagerx.Graph.create()

    # Create camera
    from example.objects.camera.objects import Camera
    import example.objects
    urdf = os.path.dirname(example.objects.__file__) + "/camera/assets/realsense2_d435.urdf"
    cam = Camera.make(
        "cam",
        rate=rate,
        sensors=["rgb", "rgba", "rgbd"],
        urdf=urdf,
        optical_link="camera_color_optical_frame",
        calibration_link="camera_bottom_screw_frame",
    )
    # graph.add(cam)

    # Create solid object
    from example.objects.solid.objects import Solid
    cube = Solid.make("cube", urdf="cube_small.urdf", rate=rate)
    graph.add(cube)

    # Create arm
    from example.objects.vx300s.objects import Vx300s
    arm = Vx300s.make(
        "viper",
        sensors=["pos", "vel", "ft", "at"],
        actuators=["joint_control", "gripper_control"],
        states=["pos", "vel", "gripper"],
        rate=rate,
        control_mode=control_mode,
    )
    graph.add(arm)

    # Connect the nodes
    graph.connect(action="joints", target=arm.actuators.joint_control)
    graph.connect(action="gripper", target=arm.actuators.gripper_control)
    graph.connect(source=arm.sensors.pos, observation="observation")
    graph.connect(source=arm.sensors.vel, observation="vel")
    graph.connect(source=arm.sensors.ft, observation="ft")
    graph.connect(source=arm.sensors.at, observation="at")

    # Define engines
    from eagerx_pybullet.engine import PybulletEngine
    engine = PybulletEngine.make(
        rate=rate, gui=False, egl=False, sync=True, real_time_factor=0, process=engine_p
    )

    # Make backend
    # from eagerx.backends.ros1 import Ros1
    # backend = Ros1.make()
    from eagerx.backends.single_process import SingleProcess
    backend = SingleProcess.make()

    # Define environment
    class TestEnv(eagerx.BaseEnv):
        def __init__(self, name, rate, graph, engine, backend, force_start):
            self.steps = 0
            super().__init__(name, rate, graph, engine, backend=backend, force_start=force_start)

        def step(self, action):
            obs = self._step(action)
            # Determine when is the episode over
            self.steps += 1
            done = self.steps > 500
            return obs, 0, done, {}

        def reset(self):
            # Reset steps counter
            self.steps = 0

            # Sample states
            states = self.state_space.sample()

            # Perform reset
            obs = self._reset(states)
            return obs

    # Initialize Environment
    env = TestEnv(name=name, rate=rate, graph=graph, engine=engine, backend=backend, force_start=True)

    # Evaluate for 30 seconds in simulation
    _, action = env.reset(), env.action_space.sample()
    for i in range(3):
        obs, reward, done, info = env.step(action)
        if done:
            _, action = env.reset(), env.action_space.sample()
            print(f"Episode {i}")
    print("\n[Finished]")


if __name__ == "__main__":
    for p in [ENV, NP]:
        for control_mode in ["position_control", "pd_control", "torque_control", "velocity_control"]:
            test_eagerx_pybullet(control_mode, p)
