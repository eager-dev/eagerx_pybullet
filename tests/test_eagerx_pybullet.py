# Environment
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph
import eagerx.nodes  # Registers butterworth_filter # noqa # pylint: disable=unused-import
import eagerx_pybullet  # Registers PybulletBridge # noqa # pylint: disable=unused-import
import examples.objects  # Registers PybulletBridge # noqa # pylint: disable=unused-import

# OTHER
import os
import pytest

NP = eagerx.process.NEW_PROCESS
ENV = eagerx.process.ENVIRONMENT


@pytest.mark.timeout(60)
@pytest.mark.parametrize("control_mode", ["position_control", "pd_control", "torque_control", "velocity_control"])
@pytest.mark.parametrize("p", [ENV, NP])
def test_eagerx_pybullet(control_mode, p):
    # Start roscore
    roscore = eagerx.initialize("eagerx_core", anonymous=True, log_level=eagerx.log.WARN)

    # Define unique name for test environment
    name = f"{control_mode}_{p}"
    bridge_p = p
    rate = 30

    # Initialize empty graph
    graph = Graph.create()

    # Create camera
    urdf = os.path.dirname(examples.objects.__file__) + "/camera/assets/realsense2_d435.urdf"
    cam = eagerx.Object.make(
        "Camera",
        "cam",
        rate=rate,
        sensors=["rgb", "rgba", "rgbd"],
        urdf=urdf,
        optical_link="camera_color_optical_frame",
        calibration_link="camera_bottom_screw_frame",
    )
    graph.add(cam)

    # Create solid object
    cube = eagerx.Object.make("Solid", "cube", urdf="cube_small.urdf", rate=rate)
    graph.add(cube)

    # Create arm
    arm = eagerx.Object.make(
        "Vx300s",
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

    # Define bridges
    bridge = eagerx.Bridge.make(
        "PybulletBridge", rate=rate, gui=False, egl=False, is_reactive=True, real_time_factor=0, process=bridge_p
    )

    # Define step function
    def step_fn(prev_obs, obs, action, steps):
        # Calculate reward
        rwd = 0
        # Determine done flag
        done = steps > 500
        # Set info:
        info = dict()
        return obs, rwd, done, info

    # Initialize Environment
    env = EagerxEnv(name=name, rate=rate, graph=graph, bridge=bridge, step_fn=step_fn)

    # Evaluate for 30 seconds in simulation
    _, action = env.reset(), env.action_space.sample()
    for i in range(3):
        obs, reward, done, info = env.step(action)
        if done:
            _, action = env.reset(), env.action_space.sample()
            print(f"Episode {i}")
    print("\n[Finished]")
    env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")
