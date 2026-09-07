#!/usr/bin/env python3

"""Select and preprocess a SolidWorks-exported rover Xacro."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import re
from pathlib import Path
import xml.etree.ElementTree as ET


RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[36m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
MAGENTA = "\033[35m"


DEFAULT_COLORS = {
    "base_link": "0.95 0.85 0.65 1",
    "shoulder_yaw_link": "0.20 0.45 0.95 1",
    "shoulder_pitch_link": "0.20 0.75 0.35 1",
    "elbow_pitch_link": "0.95 0.65 0.10 1",
    "elbow_roll_link": "0.70 0.25 0.85 1",
    "wrist_pitch_link": "0.10 0.75 0.75 1",
    "wrist_roll_link": "0.95 0.35 0.65 1",
}

WHEEL_COLOR = "0.85 0.05 0.05 1"
BRACKET_COLOR = "0.10 0.75 0.15 1"
FRAME_COLOR = "0.95 0.85 0.65 1"
COLOR_BACKLOG = [
    "0.15 0.55 0.95 1",
    "0.95 0.35 0.15 1",
    "0.20 0.75 0.35 1",
    "0.70 0.25 0.85 1",
    "0.95 0.65 0.10 1",
    "0.10 0.75 0.75 1",
    "0.95 0.35 0.65 1",
    "0.55 0.80 0.15 1",
    "0.35 0.30 0.90 1",
    "0.10 0.60 0.45 1",
    "0.80 0.20 0.45 1",
    "0.45 0.70 0.95 1",
    "0.75 0.45 0.15 1",
    "0.30 0.80 0.70 1",
    "0.60 0.40 0.20 1",
    "0.45 0.85 0.45 1",
    "0.85 0.45 0.85 1",
    "0.20 0.50 0.65 1",
    "0.90 0.55 0.35 1",
    "0.40 0.65 0.25 1",
    "0.65 0.30 0.55 1",
    "0.25 0.70 0.85 1",
    "0.85 0.75 0.20 1",
    "0.50 0.50 0.90 1",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--imports", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def available_robot_files(imports: Path) -> list[Path]:
    # Import layout rules:
    #   - Each top-level entry in URDF_Imports is one rover import.
    #   - Each import contains exactly one .urdf or .xacro robot description.
    #   - The robot description may be directly in the import or in a nested folder.
    #   - The mesh directory is discovered separately by looking for "mesh" in its name.
    robot_files = []
    for entry in imports.iterdir():
        if entry.is_dir():
            candidates = [
                path
                for pattern in ("*.urdf", "*.xacro")
                for path in entry.rglob(pattern)
                if path.is_file()
            ]
        elif entry.suffix.lower() in {".urdf", ".xacro"}:
            candidates = [entry]
        else:
            continue

        if len(candidates) > 1:
            raise RuntimeError(
                f"Import '{entry.name}' contains more than one URDF/Xacro file: {candidates}"
            )
        if candidates:
            robot_files.append(candidates[0])

    return sorted(robot_files, key=lambda path: path.stat().st_mtime, reverse=True)


def choose_robot_file(files: list[Path], imports: Path) -> Path:
    if not files:
        raise FileNotFoundError("No .xacro or .urdf files were found under the imports directory")

    print(f"\n{BOLD}{CYAN}Available rover URDF/Xacro files:{RESET}")
    for index, path in enumerate(files, start=1):
        stat = path.stat()
        timestamp = getattr(stat, "st_birthtime", stat.st_ctime)
        created = dt.datetime.fromtimestamp(timestamp).astimezone()
        entry_name = path.relative_to(imports).parts[0]
        print(
            f"  {GREEN}{index}{RESET}. {entry_name} "
            f"{YELLOW}(Date created: {created:%Y-%m-%d %H:%M:%S %Z}){RESET}"
        )

    while True:
        choice = input(
            f"{BOLD}{MAGENTA}Select a model number and press Enter: {RESET}"
        ).strip()
        try:
            index = int(choice) - 1
            if 0 <= index < len(files):
                return files[index]
        except ValueError:
            pass
        print("Please enter one of the listed numbers.")


def link_color(link_name: str) -> str | None:
    if "swerve_yaw" in link_name or "bracket" in link_name:
        return BRACKET_COLOR
    if "wheel" in link_name:
        return WHEEL_COLOR
    if link_name == "base_link":
        return FRAME_COLOR
    return DEFAULT_COLORS.get(link_name)


def load_color_assignments(path: Path) -> dict[str, str]:
    if not path.exists():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def save_color_assignments(path: Path, assignments: dict[str, str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(assignments, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def backlog_color(link_name: str, assignments: dict[str, str]) -> tuple[str, bool]:
    if link_name in assignments:
        return assignments[link_name], False

    used = set(assignments.values())
    used.update(DEFAULT_COLORS.values())
    used.update({WHEEL_COLOR, BRACKET_COLOR, FRAME_COLOR})
    for color in COLOR_BACKLOG:
        if color not in used:
            assignments[link_name] = color
            return color, True
    raise RuntimeError("The color backlog is exhausted; add more colors to COLOR_BACKLOG")


def preprocess(
    source: Path,
    imports: Path,
    destination: Path,
    color_assignments: dict[str, str],
) -> None:
    text = source.read_text(encoding="utf-8")
    # Some exporters place a comment before the XML declaration. XML requires
    # the declaration to be the first item, so move it to the front if needed.
    declaration = re.search(r"<\?xml[^?]*\?>", text)
    if declaration and declaration.start() != 0:
        text = declaration.group(0) + "\n" + text[: declaration.start()] + text[declaration.end() :]

    import_root = imports / source.relative_to(imports).parts[0]
    # The mesh directory does not have to be named exactly "meshes". The
    # importer assumes there is one directory below the import whose name
    # contains "mesh" (case-insensitive).
    mesh_dirs = [
        path
        for path in import_root.rglob("*")
        if path.is_dir() and "mesh" in path.name.lower()
    ]
    if len(mesh_dirs) != 1:
        raise RuntimeError(
            f"Import '{import_root.name}' must contain exactly one directory "
            f"with 'mesh' in its name; found: {mesh_dirs}"
        )
    mesh_dir = mesh_dirs[0]
    relative_mesh_dir = mesh_dir.relative_to(imports)
    mesh_prefix = (
        "package://urdf_viewer_pkg/URDF_Imports/"
        + relative_mesh_dir.as_posix()
        + "/"
    )
    root = ET.fromstring(text)

    mesh_count = 0
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename")
        if not filename:
            continue
        mesh.set("filename", mesh_prefix + Path(filename).name)
        mesh_count += 1

    changed_colors = 0
    assigned_colors = []
    for link in root.findall("link"):
        link_name = link.get("name", "")
        color = link_color(link_name)
        if color is None:
            color, assigned = backlog_color(link_name, color_assignments)
            if assigned:
                assigned_colors.append(link_name)
        for visual in link.findall("visual"):
            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material", {"name": f"{link_name}_material"})
            color_element = material.find("color")
            if color_element is None:
                color_element = ET.SubElement(material, "color")
            color_element.set("rgba", color)
            changed_colors += 1

    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(destination, encoding="utf-8", xml_declaration=True)
    print(f"Processed {source}")
    print(f"Rebound {mesh_count} mesh URI(s) to {mesh_prefix}")
    print(f"Recolored {changed_colors} visual element(s)")
    print("Applied colors: wheels red, wheel brackets green, base_link cream")
    print("Preserved the configured colors for the remaining known links")
    if assigned_colors:
        print(f"Assigned persistent backlog colors to: {', '.join(assigned_colors)}")
    print(f"Wrote {destination}")


def main() -> None:
    args = parse_args()
    imports = args.imports.expanduser().resolve()
    output = args.output.expanduser().resolve()
    selected = choose_robot_file(available_robot_files(imports), imports)
    assignments_path = output.parent / "color_assignments.json"
    assignments = load_color_assignments(assignments_path)
    preprocess(selected, imports, output, assignments)
    save_color_assignments(assignments_path, assignments)


if __name__ == "__main__":
    main()
