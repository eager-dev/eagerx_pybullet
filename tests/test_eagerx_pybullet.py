# ROS packages required
from eagerx import Object, Bridge, initialize, log, process

# Environment
from eagerx.core.env import EagerxEnv
from eagerx.core.graph import Graph

# Implementation specific
import eagerx.nodes  # Registers butterworth_filter # noqa # pylint: disable=unused-import
import eagerx_pybullet  # Registers PybulletBridge # noqa # pylint: disable=unused-import

import tests.objects.solid # Registers test object # noqa # pylint: disable=unused-import

import pytest

NP = process.NEW_PROCESS
ENV = process.ENVIRONMENT

@pytest.mark.parametrize(
    "eps, steps, is_reactive, rtf, p",
    [(3, 3, True, 0, ENV), (3, 3, True, 0, NP)]
)
def test_bridge(eps, steps, is_reactive, rtf, p):
    # Start roscore
    roscore = initialize("eagerx_core", anonymous=True, log_level=log.WARN)

    # Define unique name for test environment
    name = f"{eps}_{steps}_{is_reactive}_{p}"
    bridge_p = p
    rate = 30

    # Initialize empty graph
    graph = Graph.create()

    # Define bridges
    bridge = Bridge.make("PybulletBridge", rate=rate, is_reactive=is_reactive, real_time_factor=rtf, process=bridge_p)

    if roscore:
        roscore.shutdown()
    print("\n[Shutdown]")
