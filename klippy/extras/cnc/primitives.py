from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import math


class MotionType(Enum):
    """
    Enumeration of supported CNC motion types.

    These correspond directly to standard G-codes.
    """
    RAPID = "G0"
    LINEAR = "G1"
    ARC_CW = "G2"
    ARC_CCW = "G3"


@dataclass
class MotionPrimitive:
    """
    Represents a single, fully-resolved motion command.

    A MotionPrimitive is the lowest-level motion unit in the CNC pipeline.
    By the time a primitive reaches the executor, all modal state,
    geometry, and feedrate must already be resolved.
    """

    # Motion type (rapid, linear, arc-derived linear, etc.)
    motion: MotionType

    # Absolute machine-space start position
    start: Tuple[float, float, float]

    # Absolute machine-space end position
    end: Tuple[float, float, float]

    # Feedrate in units per minute (None for non-feed moves)
    feedrate: Optional[float] = None

    def length(self):
        """
        Compute the Euclidean length of the motion primitive.

        Used for:
        - Progress tracking
        - Time estimation
        """
        x0, y0, z0 = self.start
        x1, y1, z1 = self.end

        dx = x1 - x0
        dy = y1 - y0
        dz = z1 - z0

        return math.sqrt(dx*dx + dy*dy + dz*dz)
