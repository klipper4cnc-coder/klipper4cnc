# klippy/extras/cnc/executor.py

from abc import ABC, abstractmethod


class MotionExecutor(ABC):
    """
    Abstract base class for CNC motion execution backends.

    A MotionExecutor is responsible for taking fully-resolved
    motion primitives (linear moves, arcs already segmented, etc.)
    and sending them to the underlying motion system.

    This abstraction allows:
    - A real Klipper-backed executor
    - Mock executors for testing
    - Potential future simulators or loggers
    """

    @abstractmethod
    def execute(self, primitive):
        """
        Execute a single MotionPrimitive.

        The primitive is expected to have:
        - start and end positions
        - a resolved feedrate
        - a known motion type (e.g. linear)

        Implementations should NOT perform lookahead or buffering
        beyond what the backend requires.
        """
        pass

    @abstractmethod
    def flush(self):
        """
        Flush any queued motion.

        For buffered backends (like Klipper), this ensures all
        motion has been fully handed off before returning.
        """
        pass
