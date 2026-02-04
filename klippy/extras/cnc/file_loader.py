# klippy/extras/cnc/file_loader.py

class GCodeFileLoader:
    def __init__(self, filename):
        self.filename = filename

    def load(self):
        lines = []

        with open(self.filename, "r") as f:
            for raw_line in f:
                line = raw_line.strip()

                # Skip blank lines
                if not line:
                    continue

                # Strip comments (; and ())
                if ";" in line:
                    line = line.split(";", 1)[0].strip()

                if not line:
                    continue

                lines.append(line)

        return lines
