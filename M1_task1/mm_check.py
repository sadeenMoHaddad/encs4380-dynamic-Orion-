import re, sys
from collections import deque
# usage: python check_dump.py dump.txt gx0 gy0 gx1 gy1
f, gx0, gy0, gx1, gy1 = sys.argv[1], *map(int, sys.argv[2:6])
w = h = None; c = {}
for line in open(f):
    m = re.match(r'@MAP .* w=(\d+) h=(\d+)', line)
    if m: w, h = int(m[1]), int(m[2])
    m = re.match(r'@C (\d+) (\d+) (\d) (\d) (\d) (\d)', line)
    if m:
        x, y, n, e, s, wl = map(int, m.groups()); c[(x, y)] = (n, e, s, wl)
assert len(c) == w*h, "cell count"
dx, dy = (0,1,0,-1), (1,0,-1,0)      # N E S W
bad = 0
for (x, y), b in c.items():
    for d in range(4):
        nb = (x+dx[d], y+dy[d])
        if nb not in c:
            if not b[d]: print("outer wall missing", x, y, d); bad += 1
        elif b[d] != c[nb][(d+2) % 4]:
            print("asymmetric wall", x, y, d); bad += 1
print("start walls (N,E,S,W):", c[(0,0)], "(want 0,1,1,1)")
dist = {(0,0): 0}; q = deque([(0,0)])
while q:
    p = q.popleft()
    if gx0 <= p[0] <= gx1 and gy0 <= p[1] <= gy1:
        print("shortest path:", dist[p], "cells"); break
    for d in range(4):
        if not c[p][d]:
            nb = (p[0]+dx[d], p[1]+dy[d])
            if nb not in dist: dist[nb] = dist[p]+1; q.append(nb)
print("dump OK" if not bad else f"{bad} problems")
