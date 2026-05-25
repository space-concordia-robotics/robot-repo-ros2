#!/usr/bin/env python
from pathlib import Path

import math
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use("Agg")


def generate_spiral(radius=10.0, step=0.5, radius_factor=0.5):
    """
    Generates a list of points that form a spiral.

    The code in this function should closely mirror the contents of autonomy/util.hpp,
    as this file is used to prototype & visualize the spiral.

    :param radius: The outer radius of the spiral
    :param step: The distance between points in the spiral
    :param radius_factor: The growth factor for the spiral
    :return: The points & headings in the generated spiral
    """
    points = [(0, 0)]
    headings = [math.pi / 2.0]

    theta = 0.0
    angle = 0.0
    r = 0.75  # idk why, but this value seems to work well as a starting point

    while r < radius:
        dtheta = step / math.sqrt(r ** 2)
        theta += dtheta
        r = radius_factor * theta
        angle += step / r

        x = r * np.cos(theta)
        y = r * np.sin(theta)
        points.append((x, y))
        headings.append(theta + math.pi / 2.0)

    additional_rotation = theta + 1 * math.pi
    while theta < additional_rotation:
        dtheta = step / math.sqrt(r ** 2)
        theta += dtheta

        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        points.append((x, y))
        headings.append(theta + math.pi / 2.0)

    return points, headings


def main():
    radius = 15
    step = 2.5
    radius_factor = 0.5
    spiral_points, spiral_headings = generate_spiral(radius=radius, step=step, radius_factor=radius_factor)

    for idx, point in enumerate(spiral_points):
        print(f"Point {idx}: {point}")

    x_vals = [p[0] for p in spiral_points]
    y_vals = [p[1] for p in spiral_points]

    min_x, max_x = min(x_vals), max(x_vals)
    min_y, max_y = min(y_vals), max(y_vals)

    if max_x - min_x > 50 or max_y - min_y > 50:
        print("WARNING: either the x or the y axis exceed 20 units.")
        print("         this will cause a huge image to be generated")
        print("         which will consume a lot of memory.")
        print("         exiting.")
        return

    width = math.ceil(max_x - min_x) + 1
    height = math.ceil(max_y - min_y) + 1

    fig, ax = plt.subplots(figsize=(width / 2, height / 2))

    for i in range(0, len(spiral_points) - 1, 1):
        a = spiral_points[i]
        b = spiral_points[i + 1]
        plt.plot((a[0], b[0]), (a[1], b[1]), color="gray", linewidth=0.7)

    for i in range(len(spiral_points)):
        px, py = spiral_points[i]
        h = spiral_headings[i]

        arrow_len = 1.0
        dx = arrow_len * math.cos(h)
        dy = arrow_len * math.sin(h)

        plt.arrow(px, py, dx, dy, head_width=0.15, head_length=0.15, fc="red", ec="red", alpha=0.6, zorder=3)

    for x in range(math.floor(min_x) - 1, math.ceil(max_x) + 1):
        ax.axvline(x, color="gray", linewidth=0.8)
    for y in range(math.floor(min_y) - 1, math.ceil(max_y) + 1):
        ax.axhline(y, color="gray", linewidth=0.8)

    plt.scatter(x_vals, y_vals, c=range(len(spiral_points)), cmap="viridis", s=10)

    ax.set_aspect("equal")
    ax.set_xlim(math.floor(min_x) - 1, math.ceil(max_x) + 1)
    ax.set_ylim(math.floor(min_y) - 1, math.ceil(max_y) + 1)
    ax.axis("off")
    plt.savefig(Path(__file__).parent / "spiral.png", dpi=300, bbox_inches="tight", pad_inches=0)
    plt.close()


if __name__ == "__main__":
    main()
