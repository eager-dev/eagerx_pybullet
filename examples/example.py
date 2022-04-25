# Environment
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph
import eagerx.nodes  # Registers butterworth_filter # noqa # pylint: disable=unused-import
import eagerx_pybullet  # Registers PybulletBridge # noqa # pylint: disable=unused-import
import examples.objects  # Registers PybulletBridge # noqa # pylint: disable=unused-import

import os

if __name__ == "__main__":
    roscore = eagerx.initialize("eagerx_core", anonymous=True, log_level=eagerx.log.INFO)

    # Initialize empty graph
    graph = Graph.create()

    # Create camera
    cam = eagerx.Object.make(
        "Camera",
        "cam",
        rate=10.0,
        sensors=["rgb", "rgba", "rgbd"],
        urdf=os.path.dirname(examples.objects.__file__) + "/camera/assets/realsense2_d435.urdf",
        optical_link="camera_color_optical_frame",
        calibration_link="camera_bottom_screw_frame",
    )

    # Uncomment to add camera & render --> currently, rendering happens as single-process in Python so very slow....
    # graph.add(cam)
    # graph.render(source=cam.sensors.rgb, rate=10)

    # Create solid object
    cube = eagerx.Object.make("Solid", "cube", urdf="cube_small.urdf", rate=5.0)
    graph.add(cube)

    # Create arm
    arm = eagerx.Object.make(
        "Vx300s",
        "viper",
        sensors=["pos", "vel", "at"],
        actuators=["joint_control", "gripper_control"],
        states=["pos", "vel", "gripper"],
        rate=5.0,
        control_mode="position_control",
        self_collision=False,
    )
    graph.add(arm)

    # Connect the nodes
    graph.connect(action="joints", target=arm.actuators.joint_control)
    graph.connect(action="gripper", target=arm.actuators.gripper_control)
    graph.connect(source=arm.sensors.pos, observation="position")
    graph.connect(source=arm.sensors.vel, observation="velocity")
    graph.connect(source=arm.sensors.at, observation="at")

    # Show in the gui
    graph.gui()

    # Define bridgesif
    bridge = eagerx.Bridge.make(
        "PybulletBridge", rate=20.0, gui=True, egl=True, sync=True, real_time_factor=0, process=eagerx.process.NEW_PROCESS
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
    env = EagerxEnv(name="rx", rate=5.0, graph=graph, bridge=bridge, step_fn=step_fn, exclude=["at"])

    obs_space = env.observation_space

    # Render images
    env.render("human")

    # Evaluate
    for eps in range(5000):
        print(f"Episode {eps}")
        _, done = env.reset(), False
        while not done:
            action = env.action_space.sample()
            obs, reward, done, info = env.step(action)
            rgb = env.render("rgb_array")

    print("\nShutting down")
    env.shutdown()
    if roscore:
        roscore.shutdown()
    print("\nShutdown")
