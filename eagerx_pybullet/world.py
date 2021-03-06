import math as m
from eagerx.utils.utils import load


def empty_world_with_plane(bullet_client):
    # ground plane
    _ = bullet_client.loadURDF(
        "plane.urdf",
        [0, 0, 0],
        useFixedBase=True,
    )


class World:
    def __init__(self, bullet_client, gravity, world_fn, timestep):
        self._p = bullet_client
        self.gravity = gravity
        self.world_fn = world_fn
        self.timestep = timestep
        self.clean_everything()
        msg = "We only support world creation functions defined as a string of the form "
        module_type_string = f"{empty_world_with_plane.__module__}/{empty_world_with_plane.__qualname__}"
        f"'module/WorldFunctionName'. For example, '{module_type_string}'."
        if world_fn is None:
            empty_world_with_plane(bullet_client)
        elif isinstance(world_fn, str):
            try:
                load(world_fn)(bullet_client)
            except ModuleNotFoundError:
                raise ModuleNotFoundError(msg)
        else:
            raise NotImplementedError(msg)

    def clean_everything(self):
        self._p.setGravity(0, 0, self.gravity)

        # Set numSubSteps such that the physics are stepped at roughly ~240 Hz.
        numSubSteps = m.ceil(self.timestep / (1 / 240))
        self._p.setPhysicsEngineParameter(
            fixedTimeStep=self.timestep,
            numSubSteps=numSubSteps,
        )

    def step(self):
        self._p.stepSimulation()
