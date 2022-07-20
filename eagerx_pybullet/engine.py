from typing import Optional, Dict, List
import numpy as np

# RX IMPORTS
import eagerx
import eagerx.core.register as register
from eagerx.core.entities import Engine


def check_if_pybullet_is_installed():
    try:
        import pybullet
        import pybullet_data
        from pybullet_utils import bullet_client
    except ImportError as e:
        from gym import error

        raise error.DependencyNotInstalled("{}. (HINT: you need to install PyBullet)".format(e))


class PybulletEngine(Engine):
    """A engine between the pybullet physics server and EAGERx engine nodes."""

    @classmethod
    def make(
        cls,
        rate: float,
        process: Optional[int] = eagerx.NEW_PROCESS,
        sync: Optional[bool] = True,
        real_time_factor: Optional[float] = 0,
        simulate_delays: Optional[bool] = True,
        log_level: Optional[int] = eagerx.ERROR,
        states: List[str] = None,
        world_fn: Optional[str] = None,
        gui: bool = True,
        egl: bool = True,
        gravity: float = -9.81,
        physics_engine_params: Optional[Dict] = None,
    ):
        """A spec to create a PybulletEngine node that interfaces with a pybullet physics server.

        :param rate: Rate of the engine
        :param process: {0: NEW_PROCESS, 1: ENVIRONMENT, 2: ENGINE, 3: EXTERNAL}
        :param sync: Run reactive or async
        :param real_time_factor: Simulation speed. 0 == "as fast as possible".
        :param simulate_delays: Boolean flag to simulate delays.
        :param log_level: {0: SILENT, 10: DEBUG, 20: INFO, 30: WARN, 40: ERROR, 50: FATAL}
        :param states: Physics engine parameters that are to be varied over episodes as a form of domain randomization.
                       Currently available: `erp`, `contactERP`, `frictionERP`.
        :param world_fn: A string with syntax `module/WorldFnName` that received `bullet_client` as an argument. The
                         function builds-up the (static) world (i.e. loads urdfs into pybullet). See
                         `eagerx_pybullet.world/empty_world_with_plane` for an example.
        :param gui: Create a GUI connection with 3D OpenGL rendering within the same process space as PyBullet.
        :param egl: Enable hardware accelerated OpenGL rendering without a X11 context for faster headless rendering.
        :param gravity: Sets the gravity constant along the y-axis.
        :param physics_engine_params: Parameter keys with their desired value. See the pybullet documentation for more
                                      info on the physics engine parameters:
                                      https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.k37c0drzdf21

                                      .. note:: fixedTimeStep cannot be set, as it is determined by the specified rate of
                                                the engine. Per default, numSubSteps is set such that simulation steps are
                                                taken at 240 hz. Tune numSubSteps to trade performance over accuracy.

        :return: EngineSpec
        """
        spec = cls.get_specification()

        # Modify default engine params
        spec.config.rate = rate
        spec.config.process = process
        spec.config.sync = sync
        spec.config.real_time_factor = real_time_factor
        spec.config.simulate_delays = simulate_delays
        spec.config.log_level = log_level
        spec.config.color = "magenta"
        spec.config.states = states if isinstance(states, list) else []

        # Add custom params
        spec.config.world_fn = world_fn
        spec.config.gui = gui
        spec.config.egl = egl
        spec.config.gravity = gravity
        spec.config.physics_engine_params = physics_engine_params if isinstance(physics_engine_params, dict) else None
        return spec

    def initialize(self, spec):
        """Initializes the engine to pybullet.

        :param spec: the specification that is used to initialize the engine.
        """
        # Check if pybullet is installed.
        check_if_pybullet_is_installed()

        # Connect to pybullet
        self._p, self.physics_client_id = self._start_simulator(spec.config.gui, spec.config.egl)
        # Initialzize
        from eagerx_pybullet.world import World

        world = World(
            self._p,
            gravity=spec.config.gravity,
            world_fn=spec.config.world_fn,
            timestep=1 / self.rate,
        )
        # Set physics parameters
        if spec.config.physics_engine_params:
            assert "fixedTimeStep" not in spec.config.physics_engine_params, (
                "Cannot set the fixedTimeStep via the physics_engine_params. "
                f"This is determined by the engine's rate: dt = 1/{self.rate} s."
            )
            self._p.setPhysicsEngineParameter(**spec.config.physics_engine_params)
        # Create pybullet simulator that will be shared with all EngineStates & EngineNodes (if launched in same process).
        self.simulator = dict(client=self._p, world=world, robots={})

    def _start_simulator(self, gui, egl):
        import pybullet
        import pybullet_data
        from pybullet_utils import bullet_client

        if gui:
            p = bullet_client.BulletClient(connection_mode=pybullet.GUI)
        else:
            # p = bullet_client.BulletClient(pybullet.SHARED_MEMORY, options="-shared_memory_key 1234")
            p = bullet_client.BulletClient()

        physics_client_id = p._client
        p.resetSimulation()
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=1)
        # optionally enable EGL for faster headless rendering
        if egl:
            con_mode = p.getConnectionInfo()["connectionMethod"]
            if con_mode == p.DIRECT:
                import pkgutil

                egl = pkgutil.get_loader("eglRenderer")
                if egl:
                    p.loadPlugin(egl.get_filename(), "_eglRendererPlugin")
                else:
                    p.loadPlugin("eglRendererPlugin")

        # Add search path for urdfs
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=p._client)
        return p, physics_client_id

    def pre_reset(self, *args, **kwargs):
        pass

    def add_object(
        self,
        spec,
        urdf: str,
        basePosition: List = [0, 0, 0],
        baseOrientation: List = [0, 0, 0, 0],
        fixed_base: bool = True,
        self_collision: bool = False,
        globalScaling: float = 1.0,
        flags: int = 0,
    ):
        """
        Adds an object to the connected Pybullet physics server.

        :param spec: The object specificaiton of the :class:`~eagerx.core.entities.Object` that is to be added.
        :param urdf: A fullpath (ending with .urdf), a key that points to the urdf (xml)string on the rosparam server,
                     or an urdf within pybullet's search path. The `pybullet_data` package is included in the search path.
        :param basePosition: Base position of the object [x, y, z].
        :param baseOrientation: Base orientation of the object in quaternion [x, y, z, w].
        :param fixed_base: Force the base of the loaded object to be static.
        :param self_collision: Sets the `URDF_USE_SELF_COLLISION` flag to allow self collisions.
        :param globalScaling: globalScaling will apply a scale factor to the URDF model.
        :param flags: Flags (see link below) that can be combined using a bitwise OR, |.
                      See https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit# for all
                      available flags.
        """
        obj_name = spec.config.name
        entity_id = spec.config.entity_id

        # add objects to simulator (we have a ref to the simulator with self.simulator)
        self.backend.loginfo(f'Adding object "{obj_name}" of type "{entity_id}" to the simulator.')

        # Add self collision to flag
        import pybullet

        if self_collision:
            flags = flags | pybullet.URDF_USE_SELF_COLLISION
        else:
            flags = flags

        # Add object
        if urdf:
            from eagerx_pybullet.robot import URDFBasedRobot

            self.simulator["robots"][obj_name] = URDFBasedRobot(
                bullet_client=self._p,
                model_urdf=urdf,  # Can be path (ending with .urdf), or ros param key to urdf (xml)string.
                robot_name=obj_name,
                basePosition=basePosition,
                baseOrientation=baseOrientation,
                fixed_base=fixed_base,
                flags=flags,
            )
        else:  # if no urdf is provided, create dummy robot.
            self.simulator["robots"][obj_name] = None

    @register.states(
        erp=eagerx.Space(low=0.2, high=0.2, shape=()),
        contactERP=eagerx.Space(low=0.2, high=0.2, shape=()),
        frictionERP=eagerx.Space(low=0.2, high=0.2, shape=()),
    )
    def reset(self, erp: np.float = None, contactERP: np.float = None, frictionERP: np.float = None):
        """Set any of the physics engine parameters (registered as states) if they were selected."""
        physics_engine_params = {}
        if erp:
            physics_engine_params["erp"] = erp
        if contactERP:
            physics_engine_params["contactERP"] = contactERP
        if frictionERP:
            physics_engine_params["frictionERP"] = frictionERP
        if len(physics_engine_params) > 0:
            self._p.setPhysicsEngineParameter(**physics_engine_params)

    def callback(self, t_n: float):
        """Here, we step the world by 1/rate seconds."""
        self.simulator["world"].step()

    def shutdown(self) -> None:
        """Disconnects the engine from the pybullet physics server"""
        self._p.disconnect()
