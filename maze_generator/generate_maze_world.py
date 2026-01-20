#!/usr/bin/env python3

import argparse
import random
import heapq
from xml.etree.ElementTree import Element, SubElement, ElementTree
from PIL import Image, ImageDraw

# ---------------- Parameters ----------------

CELL_SIZE = 1.0
WALL_THICKNESS = 0.1
WALL_HEIGHT = 1.0
PIXELS_PER_METER = 100

# ---------------- Maze Structures ----------------

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

    # Neighbors generator
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

    # Remove wall between two cells
    def remove_wall(self, c1, c2, d):
        opp = {"N": "S", "S": "N", "E": "W", "W": "E"}
        c1.walls[d] = False
        c2.walls[opp[d]] = False

    # DFS maze generation
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

    # Prim's maze generation
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

    # Add entry/exit
    def add_entry_exit(self, entry, exit):
        er, ec, edir = entry
        xr, xc, xdir = exit
        self.grid[er][ec].walls[edir] = False
        self.grid[xr][xc].walls[xdir] = False

    # Add complete outer boundary walls
    def add_outer_walls(self):
        for c in range(self.cols):
            self.grid[0][c].walls["N"] = True
            self.grid[self.rows-1][c].walls["S"] = True
        for r in range(self.rows):
            self.grid[r][0].walls["W"] = True
            self.grid[r][self.cols-1].walls["E"] = True

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
    pose.text = f"{x} {y} {WALL_HEIGHT/2} 0 0 0"

    link = SubElement(model, "link", {"name": "link"})
    for tag in ["collision", "visual"]:
        obj = SubElement(link, tag, {"name": tag})
        geom = SubElement(obj, "geometry")
        box = SubElement(geom, "box")
        size = SubElement(box, "size")
        size.text = f"{sx} {sy} {WALL_HEIGHT}"

def save_world(maze, filename):
    sdf = Element("sdf", {"version": "1.6"})
    world = SubElement(sdf, "world", {"name": "maze"})

    for r in range(maze.rows):
        for c in range(maze.cols):
            cell = maze.grid[r][c]
            x0, y0 = cell_center_world(maze, r, c)
            x0 -= CELL_SIZE / 2
            y0 -= CELL_SIZE / 2

            # North wall
            if cell.walls["N"]:
                boundary = (r == 0)
                add_wall(world, r, c, "N", x0 + CELL_SIZE/2, y0 + CELL_SIZE,
                         CELL_SIZE, WALL_THICKNESS, boundary)
            # West wall
            if cell.walls["W"]:
                boundary = (c == 0)
                add_wall(world, r, c, "W", x0, y0 + CELL_SIZE/2,
                         WALL_THICKNESS, CELL_SIZE, boundary)
            # South wall (only bottom row)
            if cell.walls["S"] and r == maze.rows-1:
                boundary = True
                add_wall(world, r, c, "S", x0 + CELL_SIZE/2, y0,
                         CELL_SIZE, WALL_THICKNESS, boundary)
            # East wall (only rightmost column)
            if cell.walls["E"] and c == maze.cols-1:
                boundary = True
                add_wall(world, r, c, "E", x0 + CELL_SIZE, y0 + CELL_SIZE/2,
                         WALL_THICKNESS, CELL_SIZE, boundary)

    ElementTree(sdf).write(filename)
    print(f"[OK] Gazebo world saved: {filename}")

# ---------------- Images ----------------

def draw_arrow(draw, px, py, d, color):
    s = 8
    if d == "N":
        pts = [(px, py-s), (px-s, py+s), (px+s, py+s)]
    elif d == "S":
        pts = [(px, py+s), (px-s, py-s), (px+s, py-s)]
    elif d == "E":
        pts = [(px+s, py), (px-s, py-s), (px-s, py+s)]
    else:
        pts = [(px-s, py), (px+s, py-s), (px+s, py+s)]
    draw.polygon(pts, fill=color)

def save_top_view(maze, entry, exit, path, filename):
    w = maze.cols * CELL_SIZE * PIXELS_PER_METER
    h = maze.rows * CELL_SIZE * PIXELS_PER_METER
    img = Image.new("RGB", (int(w), int(h)), "white")
    draw = ImageDraw.Draw(img)

    for r in range(maze.rows):
        for c in range(maze.cols):
            cell = maze.grid[r][c]
            x0, y0 = cell_center_world(maze, r, c)
            x0 -= CELL_SIZE / 2
            y0 -= CELL_SIZE / 2

            if cell.walls["N"]:
                draw.line([world_to_px(x0, y0+CELL_SIZE, maze),
                           world_to_px(x0+CELL_SIZE, y0+CELL_SIZE, maze)],
                          fill="black", width=3)
            if cell.walls["W"]:
                draw.line([world_to_px(x0, y0, maze),
                           world_to_px(x0, y0+CELL_SIZE, maze)],
                          fill="black", width=3)
            # Draw South wall of bottom row
            if cell.walls["S"] and r == maze.rows-1:
                draw.line([world_to_px(x0, y0, maze),
                           world_to_px(x0+CELL_SIZE, y0, maze)],
                          fill="black", width=3)
            # Draw East wall of rightmost column
            if cell.walls["E"] and c == maze.cols-1:
                draw.line([world_to_px(x0+CELL_SIZE, y0, maze),
                           world_to_px(x0+CELL_SIZE, y0+CELL_SIZE, maze)],
                          fill="black", width=3)

    if path:
        pts = [world_to_px(*cell_center_world(maze, r, c), maze) for r, c in path]
        draw.line(pts, fill="red", width=4)

    ex, ey = cell_center_world(maze, entry[0], entry[1])
    gx, gy = cell_center_world(maze, exit[0], exit[1])

    draw_arrow(draw, *world_to_px(ex, ey, maze), entry[2], "green")
    draw_arrow(draw, *world_to_px(gx, gy, maze), exit[2], "blue")

    img.save(filename)
    print(f"[OK] Image saved: {filename}")

# ---------------- Main ----------------

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--rows", type=int, required=True)
    p.add_argument("--cols", type=int, required=True)
    p.add_argument("--algorithm", choices=["dfs", "prim"], required=True)
    p.add_argument("--output", required=True)
    args = p.parse_args()

    maze = Maze(args.rows, args.cols)
    if args.algorithm == "dfs":
        maze.generate_dfs()
    else:
        maze.generate_prim()

    entry = (0, 0, "W")
    exit = (args.rows-1, args.cols-1, "E")

    maze.add_outer_walls()       # enforce full boundary
    maze.add_entry_exit(entry, exit)

    path = astar(maze, (entry[0], entry[1]), (exit[0], exit[1]))

    save_world(maze, args.output)
    save_top_view(maze, entry, exit, None,
                  args.output.replace(".world", "_topview.png"))
    save_top_view(maze, entry, exit, path,
                  args.output.replace(".world", "_solution.png"))

if __name__ == "__main__":
    main()
