# klippy/extras/cnc/parser.py

import re

# Regular expression matching a G-code "word":
#   <LETTER><NUMBER>
# Examples: X10, Y-2.5, G1, M3
WORD_RE = re.compile(r'([A-Z])([-+]?[0-9]*\.?[0-9]+)')


def parse_gcode_line(line):
    """
    Parse a single G-code line into structured components.

    The returned structure separates:
    - Axis and parameter words (X, Y, Z, I, J, F, etc.)
    - G-codes (modal motion / state commands)
    - M-codes (machine control commands)

    Returns:
        {
            "words": { "X": 1.0, "Y": 2.0, ... },
            "gcodes": [0, 1, 2, ...],
            "mcodes": [3, 5, ...],
        }
    """

    # Remove semicolon comments
    line = line.split(';')[0]

    # Remove parenthesis-style comments
    line = re.sub(r'\(.*?\)', '', line)

    # Normalize whitespace and case
    line = line.strip().upper()

    # Empty or comment-only line
    if not line:
        return None

    words = {}
    gcodes = []
    mcodes = []

    # Extract all letter/value pairs in the line
    for letter, value in WORD_RE.findall(line):
        val = float(value)

        if letter == "G":
            # Modal or non-modal G-code
            gcodes.append(int(val))
        elif letter == "M":
            # Machine-specific command
            mcodes.append(int(val))
        else:
            # Axis, feedrate, or other parameter
            words[letter] = val

    return {
        "words": words,
        "gcodes": gcodes,
        "mcodes": mcodes,
    }
