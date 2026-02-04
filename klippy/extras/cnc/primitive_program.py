class PrimitiveProgram:
    def __init__(self, primitives):
        self.primitives = primitives
        self.index = 0

    def has_next(self):
        return self.index < len(self.primitives)

    def next(self):
        p = self.primitives[self.index]
        self.index += 1
        return p

    def current(self):
        return self.index

    def total(self):
        return len(self.primitives)

    def percent_complete(self):
        if self.total() == 0:
            return 100.0
        return (self.index / self.total()) * 100.0

    def remaining_time_seconds(self):
        # placeholder: executor-based or feedrate-based later
        return 0
