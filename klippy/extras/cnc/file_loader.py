# klippy/extras/cnc/file_loader.py

class GCodeFileLoader:
    """
    Simple loader for G-code files.

    This class reads a G-code file from disk and returns a cleaned
    list of executable G-code lines by:
    - Removing blank lines
    - Stripping comments
    """

    def __init__(self, filename):
        # Path to the G-code (.nc / .gcode) file
        self.filename = filename

    def load(self):
        """
        Load and preprocess the G-code file.

        Returns a list of non-empty, comment-free G-code lines.
        """
        lines = []

        with open(self.filename, "r") as f:
            for raw_line in f:
                # Remove leading/trailing whitespace
                line = raw_line.strip()

                # Skip blank lines
                if not line:
                    continue

                # Strip semicolon-style comments
                # (Inline comments after code are also removed)
                if ";" in line:
                    line = line.split(";", 1)[0].strip()

                # Line may become empty after stripping comments
                if not line:
                    continue

                # Store cleaned G-code line
                lines.append(line)

        return lines
