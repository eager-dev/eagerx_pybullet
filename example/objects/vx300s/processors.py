import numpy as np
from eagerx.core.entities import Processor
from eagerx.core.specs import ProcessorSpec


class MirrorAction(Processor):
    @classmethod
    def make(cls, index=0, offset=0) -> ProcessorSpec:
        spec = cls.get_specification()
        spec.config.update(offset=offset, index=index)
        return spec

    def initialize(self, spec: ProcessorSpec) -> None:
        self.offset = spec.config.offset
        self.index = spec.config.index

    def convert(self, msg: np.ndarray) -> np.ndarray:
        action = msg[self.index]
        mirrored = np.array([action, -action], dtype="float32")
        mirrored += self.offset
        return mirrored
