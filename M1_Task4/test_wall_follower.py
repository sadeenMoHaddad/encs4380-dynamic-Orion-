
import sys

filename = sys.argv[1] if len(sys.argv) > 1 else "maze8_walls.map"

with open(filename) as f:
    lines = [line.rstrip("\n") for line in f]

n = (len(lines) - 1) // 2

# Directions: North, East, South, West
dx = [0, 1, 0, -1]
dy = [1, 0, -1, 0]

walls = {}

for y in range(n):
    for x in range(n):
        top = lines[2 * (n - y - 1)]
        middle = lines[2 * (n - y - 1) + 1]
        bottom = lines[2 * (n - y)]

        walls[(x, y)] = [
            top[1 + 4*x:4 + 4*x] == "---",  # N
            middle[4*(x+1)] == "|",         # E
            bottom[1 + 4*x:4 + 4*x] == "---",# S
            middle[4*x] == "|"              # W
        ]

# Center 2x2 goal
goal = {
    (n//2 - 1, n//2 - 1),
    (n//2,     n//2 - 1),
    (n//2 - 1, n//2),
    (n//2,     n//2)
}

def test(rule):
    x, y, heading = 0, 0, 0  # Start facing North
    visited = {}
    moves = 0

    while True:
        if (x, y) in goal:
            print(rule, ": REACHED GOAL!")
            return

        state = (x, y, heading)

        if state in visited:
            cycle = moves - visited[state]
            print(rule, ": LOOP detected!")
            print("Repeated position:", state)
            print("Loop length:", cycle, "moves")
            return

        visited[state] = moves

        # Left-hand or right-hand priority
        order = [-1, 0, 1, 2] if rule == "Left" else [1, 0, -1, 2]

        for turn in order:
            direction = (heading + turn) % 4

            if not walls[(x, y)][direction]:
                x += dx[direction]
                y += dy[direction]
                heading = direction
                moves += 1
                break
        else:
            print(rule, ": STUCK!")
            return

test("Left")
test("Right")
