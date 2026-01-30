#!/usr/bin/env python3

import argparse
import random
import heapq
from xml.etree.ElementTree import Element, SubElement, ElementTree
from PIL import Image, ImageDraw

# ---------------- Parameters ----------------

CELL_SIZE = 1.0
MIN_WALL_THICKNESS = 0.06
MAX_WALL_THICKNESS = 0.16
WALL_HEIGHT = 1.0
PIXELS_PER_METER = 100

# ---------------- Maze Structures ----------------

def random_wall_thickness():
    return random.uniform(MIN_WALL_THICKNESS, MAX_WALL_THICKNESS)
class Cell:
    def __init__(self, r, c):
        self.r = r
        self.c = c
        self.walls = {"N": True, "S": True, "E": True, "W": True}
        self.visited = False


class Maze:
    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = [[Cell(r, c) for c in range(cols)] for r in range(rows)]

    def neighbors(self, cell):
        dirs = {
            "N": (cell.r - 1, cell.c),
            "S": (cell.r + 1, cell.c),
            "E": (cell.r, cell.c + 1),
            "W": (cell.r, cell.c - 1),
        }
        for d, (r, c) in dirs.items():
            if 0 <= r < self.rows and 0 <= c < self.cols:
                yield d, self.grid[r][c]

    def remove_wall(self, c1, c2, d):
        opp = {"N": "S", "S": "N", "E": "W", "W": "E"}
        c1.walls[d] = False
        c2.walls[opp[d]] = False

    def generate_dfs(self):
        stack = []
        cur = self.grid[0][0]
        cur.visited = True
        while True:
            unvisited = [(d, n) for d, n in self.neighbors(cur) if not n.visited]
            if unvisited:
                d, nxt = random.choice(unvisited)
                self.remove_wall(cur, nxt, d)
                stack.append(cur)
                nxt.visited = True
                cur = nxt
            elif stack:
                cur = stack.pop()
            else:
                break

    def generate_prim(self):
        start = self.grid[0][0]
        start.visited = True
        walls = [(start, n, d) for d, n in self.neighbors(start)]
        while walls:
            c1, c2, d = random.choice(walls)
            walls.remove((c1, c2, d))
            if not c2.visited:
                self.remove_wall(c1, c2, d)
                c2.visited = True
                for nd, nn in self.neighbors(c2):
                    if not nn.visited:
                        walls.append((c2, nn, nd))

    def add_entry_exit(self, entry, exit):
        er, ec, edir = entry
        xr, xc, xdir = exit
        self.grid[er][ec].walls[edir] = False
        self.grid[xr][xc].walls[xdir] = False

    def add_outer_walls(self):
        for c in range(self.cols):
            self.grid[0][c].walls["N"] = True
            self.grid[self.rows - 1][c].walls["S"] = True
        for r in range(self.rows):
            self.grid[r][0].walls["W"] = True
            self.grid[r][self.cols - 1].walls["E"] = True

# ---------------- A* Solver ----------------

def astar(maze, start, goal):
    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    pq = [(0, start)]
    came = {}
    g = {start: 0}

    while pq:
        _, cur = heapq.heappop(pq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]

        r, c = cur
        cell = maze.grid[r][c]
        moves = []
        if not cell.walls["N"]: moves.append((r - 1, c))
        if not cell.walls["S"]: moves.append((r + 1, c))
        if not cell.walls["E"]: moves.append((r, c + 1))
        if not cell.walls["W"]: moves.append((r, c - 1))

        for n in moves:
            ng = g[cur] + 1
            if ng < g.get(n, 1e9):
                g[n] = ng
                came[n] = cur
                heapq.heappush(pq, (ng + h(n, goal), n))
    return None

# ---------------- Coordinate Helpers ----------------

def cell_center_world(maze, r, c):
    x = c * CELL_SIZE + CELL_SIZE / 2
    y = (maze.rows - 1 - r) * CELL_SIZE + CELL_SIZE / 2
    return x, y

def world_to_px(x, y, maze):
    return (
        int(x * PIXELS_PER_METER),
        int((maze.rows * CELL_SIZE - y) * PIXELS_PER_METER),
    )

# ---------------- Gazebo World ----------------

def add_wall(world, r, c, direction, x, y, sx, sy, boundary=False):
    suffix = "_b" if boundary else ""
    model_name = f"wall_r{r}_c{c}_{direction}{suffix}"
    model = SubElement(world, "model", {"name": model_name})
    SubElement(model, "static").text = "true"

    pose = SubElement(model, "pose")
    pose.text = f"{x} {y} {WALL_HEIGHT / 2} 0 0 0"

    link = SubElement(model, "link", {"name": "link"})
    for tag in ["collision", "visual"]:
        obj = SubElement(link, tag, {"name": tag})
        geom = SubElement(obj, "geometry")
        box = SubElement(geom, "box")
        size = SubElement(box, "size")
        size.text = f"{sx} {sy} {WALL_HEIGHT}"

def add_marker(world, name, x, y, color):
    model = SubElement(world, "model", {"name": name})
    SubElement(model, "static").text = "true"

    pose = SubElement(model, "pose")
    pose.text = f"{x} {y} 0.05 0 0 0"

    link = SubElement(model, "link", {"name": "link"})
    visual = SubElement(link, "visual", {"name": "visual"})
    geom = SubElement(visual, "geometry")
    sphere = SubElement(geom, "sphere")
    SubElement(sphere, "radius").text = "0.15"

    material = SubElement(visual, "material")
    SubElement(material, "ambient").text = color
    SubElement(material, "diffuse").text = color

def save_world(maze, filename):
    sdf = Element("sdf", {"version": "1.6"})
    world = SubElement(sdf, "world", {"name": "maze"})

    include = SubElement(world, "include")
    uri = SubElement(include, "uri")
    uri.text = "model://ground_plane"

    for r in range(maze.rows):
        for c in range(maze.cols):
            cell = maze.grid[r][c]
            x0, y0 = cell_center_world(maze, r, c)
            x0 -= CELL_SIZE / 2
            y0 -= CELL_SIZE / 2

            if cell.walls["N"]:
                add_wall(world, r, c, "N",
                         x0 + CELL_SIZE / 2, y0 + CELL_SIZE,
                         CELL_SIZE, random_wall_thickness(), r == 0)

            if cell.walls["W"]:
                add_wall(world, r, c, "W",
                         x0, y0 + CELL_SIZE / 2,
                         random_wall_thickness(), CELL_SIZE, c == 0)

            if cell.walls["S"] and r == maze.rows - 1:
                add_wall(world, r, c, "S",
                         x0 + CELL_SIZE / 2, y0,
                         CELL_SIZE, random_wall_thickness(), True)

            if cell.walls["E"] and c == maze.cols - 1:
                add_wall(world, r, c, "E",
                         x0 + CELL_SIZE, y0 + CELL_SIZE / 2,
                         random_wall_thickness(), CELL_SIZE, True)

    # ---- Start & Goal Markers ----
    sx, sy = cell_center_world(maze, 0, 0)
    gx, gy = cell_center_world(maze, maze.rows - 1, maze.cols - 1)
    add_marker(world, "start_point", sx, sy, "0 1 0 1")
    add_marker(world, "goal_point", gx, gy, "1 0 0 1")

    ElementTree(sdf).write(filename)
    print(f"[OK] Gazebo world saved: {filename}")

# ---------------- Main ----------------

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--rows", type=int, required=True)
    p.add_argument("--cols", type=int, required=True)
    p.add_argument("--algorithm", choices=["dfs", "prim"], required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()

    maze = Maze(args.rows, args.cols)
    maze.generate_dfs() if args.algorithm == "dfs" else maze.generate_prim()

    entry = (0, 0, "W")
    exit = (args.rows - 1, args.cols - 1, "E")

    maze.add_outer_walls()
    maze.add_entry_exit(entry, exit)

    save_world(maze, args.output)

if __name__ == "__main__":
    main()
