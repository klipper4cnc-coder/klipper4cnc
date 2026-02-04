from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import math


class MotionType(Enum):
    RAPID = "G0"
    LINEAR = "G1"
    ARC_CW = "G2"
    ARC_CCW = "G3"


@dataclass
class MotionPrimitive:
    motion: MotionType
    start: Tuple[float, float, float]
    end: Tuple[float, float, float]
    feedrate: Optional[float] = None


    def length(self):
        x0, y0, z0 = self.start
        x1, y1, z1 = self.end

        dx = x1 - x0
        dy = y1 - y0
        dz = z1 - z0

        return math.sqrt(dx*dx + dy*dy + dz*dz)
