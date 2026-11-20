#!/usr/bin/env python3
"""Convert WGS84 coordinates (DMS/DDM) to decimal degrees."""

import re
import sys


def to_decimal(degrees, minutes, seconds, direction):
    decimal = abs(degrees) + minutes / 60 + seconds / 3600
    if direction in ("S", "W"):
        decimal = -decimal
    return decimal


def parse_coordinate(text):
    text = text.strip().upper().replace(",", " ")

    direction = None
    if text and text[0] in "NSEW":
        direction = text[0]
        text = text[1:].strip()
    elif text and text[-1] in "NSEW":
        direction = text[-1]
        text = text[:-1].strip()

    if direction is None:
        raise ValueError(f"Missing direction (N/S/E/W): {text}")

    numbers = re.findall(r"\d+\.?\d*", text)
    if not numbers:
        raise ValueError(f"No numbers found in coordinate: {text}")

    if len(numbers) == 1:
        value = float(numbers[0])
        return -abs(value) if direction in ("S", "W") and value > 0 else value

    degrees = int(numbers[0])
    minutes = float(numbers[1])
    seconds = float(numbers[2]) if len(numbers) >= 3 else 0.0
    return to_decimal(degrees, minutes, seconds, direction)


def convert_pair(text):
    text = text.strip().upper()

    match = re.match(r"^([NS].+?)\s+([WE].+)$", text)
    if match:
        return parse_coordinate(match.group(1)), parse_coordinate(match.group(2))

    match = re.match(r"^(.+[NS])\s+(.+[EW])$", text)
    if match:
        return parse_coordinate(match.group(1)), parse_coordinate(match.group(2))

    parts = text.split(None, 1)
    if len(parts) == 2:
        return parse_coordinate(parts[0]), parse_coordinate(parts[1])

    raise ValueError("Provide latitude and longitude")


def main():
    if len(sys.argv) > 1:
        text = " ".join(sys.argv[1:])
    else:
        print("Enter coordinates (lat lon), e.g. 45°52'30.5\"N 73°34'15.2\"W")
        text = input("> ")

    lat, lon = convert_pair(text)
    print(f"Latitude:  {lat:.8f}")
    print(f"Longitude: {lon:.8f}")
    print(f"{lat:.8f}, {lon:.8f}")


if __name__ == "__main__":
    main()
