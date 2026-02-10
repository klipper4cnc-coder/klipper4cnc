# klippy/extras/cnc/streamer.py


class GCodeStreamer:
    """
    Incremental G-code file streamer.

    This class provides line-by-line access to a G-code file,
    allowing CNC jobs to be streamed instead of fully loaded
    into memory at once.
    """

    def __init__(self, filepath):
        # Path to the G-code file
        self.filepath = filepath

        # File handle (opened lazily)
        self.file = None

        # Current line number (used for resume support)
        self.line_number = 0

        # End-of-file flag
        self.eof = False

    def open(self):
        """
        Open the G-code file for streaming.

        If streaming was previously interrupted, this will seek
        forward to the last processed line.
        """
        if self.file is None:
            self.file = open(self.filepath, "r")

            # Skip lines that were already processed
            for _ in range(self.line_number):
                self.file.readline()

    def close(self):
        """
        Close the G-code file if it is open.
        """
        if self.file:
            self.file.close()
            self.file = None

    def next_line(self):
        """
        Return the next executable G-code line.

        Skips:
        - Blank lines
        - Full-line comments

        Returns None when end-of-file is reached.
        """
        if self.eof or not self.file:
            return None

        while True:
            line = self.file.readline()

            if not line:
                # End of file reached
                self.eof = True
                return None

            self.line_number += 1
            line = line.strip()

            # Skip blank lines and comment-only lines
            if not line or line.startswith(";") or line.startswith("("):
                continue

            return line
