# klippy/extras/cnc/executor.py

from abc import ABC, abstractmethod

class MotionExecutor(ABC):
    @abstractmethod
    def execute(self, primitive):
        """
        Execute a single MotionPrimitive.
        """
        pass

    @abstractmethod
    def flush(self):
        """
        Flush queued motion (if applicable).
        """
        pass
