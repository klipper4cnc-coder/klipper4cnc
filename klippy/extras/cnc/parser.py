# klippy/extras/cnc/parser.py

import re

WORD_RE = re.compile(r'([A-Z])([-+]?[0-9]*\.?[0-9]+)')

def parse_gcode_line(line):
    """
    Parse a G-code line into structured components.
    Returns:
        {
            "words": { "X": 1.0, "Y": 2.0, ... },
            "gcodes": [0, 1, 2, ...],
            "mcodes": [3, 5, ...],
        }
    """
    # Remove comments
    line = line.split(';')[0]
    line = re.sub(r'\(.*?\)', '', line)
    line = line.strip().upper()

    if not line:
        return None

    words = {}
    gcodes = []
    mcodes = []

    for letter, value in WORD_RE.findall(line):
        val = float(value)

        if letter == "G":
            gcodes.append(int(val))
        elif letter == "M":
            mcodes.append(int(val))
        else:
            words[letter] = val

    return {
        "words": words,
        "gcodes": gcodes,
        "mcodes": mcodes,
    }
