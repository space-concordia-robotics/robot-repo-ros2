# ruff: noqa: D100, D103, INP001, S603, S607
import json
import subprocess
from typing import Any

# this is absolutely disgusting.
# I need it to work around the fact that when
# running python code with blender it cannot import cv2.

python_code = """
import json

import cv2.aruco as aruco
import math

aruco_bits = []

d = aruco.getPredefinedDictionary(aruco.DICT_{name}_1000)
size = d.markerSize
bytes_needed = math.ceil(size * size / 8)

for tag_id in range(d.bytesList.shape[0]):
    aruco_bits.append(d.getBitsFromByteList(d.bytesList[tag_id][0, :math.ceil(size * size / 8)], size).tolist())

print(json.dumps(aruco_bits))
"""


def get_aruco_tags(dictionary: str) -> Any:
    result = subprocess.run(
        ["python3", "-c", python_code.format(name=dictionary.upper())],
        capture_output=True,
        text=True,
        check=True,
    )
    return json.loads(result.stdout)
