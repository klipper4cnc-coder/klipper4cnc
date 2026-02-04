class CNCProgram:
    def __init__(self):
        self.primitives = []
        self.cursor = 0

    def load_primitives(self, primitives):
        self.primitives.extend(primitives)

    def has_next(self):
        return self.cursor < len(self.primitives)

    def next(self):
        p = self.primitives[self.cursor]
        self.cursor += 1
        return p

    def reset(self):
        self.primitives.clear()
        self.cursor = 0

    # ---- progress helpers ----

    def total(self):
        return len(self.primitives)

    def current(self):
        return self.cursor

    def percent_complete(self):
        if not self.primitives:
            return 0.0
        return (self.cursor / len(self.primitives)) * 100.0

    def total_time_seconds(self):
        total = 0.0
        for p in self.primitives:
            if p.feedrate and p.feedrate > 0:
                total += p.length() / (p.feedrate / 60.0)
        return total

    def remaining_time_seconds(self):
        remaining = 0.0
        for p in self.primitives[self.cursor:]:
            if p.feedrate and p.feedrate > 0:
                remaining += p.length() / (p.feedrate / 60.0)
        return remaining