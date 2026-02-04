class GCodeStreamer:
    def __init__(self, filepath):
        self.filepath = filepath
        self.file = None
        self.line_number = 0
        self.eof = False

    def open(self):
        if self.file is None:
            self.file = open(self.filepath, "r")
            for _ in range(self.line_number):
                self.file.readline()

    def close(self):
        if self.file:
            self.file.close()
            self.file = None

    def next_line(self):
        if self.eof or not self.file:
            return None

        while True:
            line = self.file.readline()
            if not line:
                self.eof = True
                return None

            self.line_number += 1
            line = line.strip()

            # Skip blanks and comments
            if not line or line.startswith(";") or line.startswith("("):
                continue

            return line
