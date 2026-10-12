
import heapq
import time
import random
import matplotlib.pyplot as plt
import streamlit as st
from matplotlib.colors import ListedColormap
from matplotlib.patches import Patch

st.set_page_config(page_title="Warehouse Robot Route Optimizer", page_icon="🤖", layout="wide")

st.title("🤖 Warehouse Robot Route Optimizer")
st.caption("Interactive A* pathfinding simulator • 4-direction movement • dynamic obstacle re-routing")

if "blocked" not in st.session_state:
    st.session_state.blocked = set()
if "start" not in st.session_state:
    st.session_state.start = (1, 1)
if "goal" not in st.session_state:
    st.session_state.goal = (18, 18)
if "grid_size" not in st.session_state:
    st.session_state.grid_size = 20
if "last_path" not in st.session_state:
    st.session_state.last_path = []
if "last_stats" not in st.session_state:
    st.session_state.last_stats = {}

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(rows, cols, start, goal, blocked):
    t0 = time.perf_counter()
    open_heap = [(manhattan(start, goal), 0, start)]
    g_cost = {start: 0}
    parent = {}
    closed = set()
    explored = 0
    while open_heap:
        f, g, cur = heapq.heappop(open_heap)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in parent:
                cur = parent[cur]
                path.append(cur)
            path.reverse()
            return path, explored, (time.perf_counter() - t0) * 1000
        closed.add(cur)
        explored += 1
        r, c = cur
        for nb in ((r-1,c), (r+1,c), (r,c-1), (r,c+1)):
            nr, nc = nb
            if not (0 <= nr < rows and 0 <= nc < cols) or nb in blocked or nb in closed:
                continue
            new_g = g_cost[cur] + 1
            if new_g < g_cost.get(nb, float("inf")):
                g_cost[nb] = new_g
                parent[nb] = cur
                heapq.heappush(open_heap, (new_g + manhattan(nb, goal), new_g, nb))
    return [], explored, (time.perf_counter() - t0) * 1000

def reset_layout(n):
    st.session_state.blocked = set()
    st.session_state.start = (0, 0)
    st.session_state.goal = (n-1, n-1)
    st.session_state.last_path = []
    st.session_state.last_stats = {}

def make_random_obstacles(n, density, seed):
    rng = random.Random(seed)
    cells = [(r,c) for r in range(n) for c in range(n)
             if (r,c) not in (st.session_state.start, st.session_state.goal)]
    rng.shuffle(cells)
    count = int(len(cells) * density / 100)
    st.session_state.blocked = set(cells[:count])
    st.session_state.last_path = []

with st.sidebar:
    st.header("Warehouse controls")
    new_size = st.slider("Grid size", 8, 40, st.session_state.grid_size)
    if new_size != st.session_state.grid_size:
        st.session_state.grid_size = new_size
        reset_layout(new_size)
    n = st.session_state.grid_size
    density = st.slider("Obstacle density (%)", 0, 40, 18, 1)
    seed = st.number_input("Random layout seed", min_value=0, max_value=99999, value=7, step=1)
    a, b = st.columns(2)
    with a:
        if st.button("Generate layout", use_container_width=True):
            make_random_obstacles(n, density, int(seed))
    with b:
        if st.button("Clear shelves", use_container_width=True):
            st.session_state.blocked = set()
            st.session_state.last_path = []
    st.divider()
    st.subheader("Edit the warehouse")
    st.write("Use Generate layout to create shelves, or Add barrier to place a row of blocked cells. Start and destination cannot be blocked.")
    quick = st.columns(2)
    with quick[0]:
        if st.button("Add barrier"):
            mid = n // 2
            st.session_state.blocked.update((mid, c) for c in range(2, n-2))
            st.session_state.last_path = []
    with quick[1]:
        if st.button("Clear barrier"):
            mid = n // 2
            st.session_state.blocked.difference_update((mid, c) for c in range(2, n-2))
            st.session_state.last_path = []
    st.divider()
    if st.button("Reset warehouse", use_container_width=True):
        reset_layout(n)

n = st.session_state.grid_size
start, goal, blocked = st.session_state.start, st.session_state.goal, st.session_state.blocked
path, explored, elapsed = astar(n, n, start, goal, blocked)
st.session_state.last_path = path
st.session_state.last_stats = {"explored": explored, "elapsed": elapsed, "length": max(0, len(path)-1)}

# Grid legend + metrics
m1, m2, m3, m4 = st.columns(4)
m1.metric("Path length", f"{len(path)-1} moves" if path else "No route")
m2.metric("Nodes explored", explored)
m3.metric("Search time", f"{elapsed:.3f} ms")
m4.metric("Blocked cells", len(blocked))

left, right = st.columns([1.7, 1])
with left:
    st.subheader("Warehouse grid")
    st.markdown("**Legend:** 🟦 Start · 🟧 Destination · ⬛ Shelves · 🟩 A* route · ⬜ Free space")
    grid = [[0 for _ in range(n)] for _ in range(n)]
    for r,c in blocked:
        grid[r][c] = 1
    for r,c in path:
        if (r,c) not in (start, goal):
            grid[r][c] = 2
    grid[start[0]][start[1]] = 3
    grid[goal[0]][goal[1]] = 4
    cmap = ListedColormap(["#f2f4f7", "#454b54", "#42b883", "#1f8fff", "#ffad33"])
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(grid, cmap=cmap, vmin=0, vmax=4)
    ax.set_xticks(range(n))
    ax.set_yticks(range(n))
    ax.set_xticklabels(range(n), fontsize=max(5, 9-n//8))
    ax.set_yticklabels(range(n), fontsize=max(5, 9-n//8))
    ax.set_xticks([x-.5 for x in range(1,n)], minor=True)
    ax.set_yticks([y-.5 for y in range(1,n)], minor=True)
    ax.grid(which="minor", color="#c5cbd3", linewidth=.45)
    ax.set_xlabel("Column")
    ax.set_ylabel("Row")
    ax.set_title("A* shortest feasible route")
    ax.legend(handles=[
        Patch(color="#1f8fff", label="Robot start"),
        Patch(color="#ffad33", label="Destination"),
        Patch(color="#454b54", label="Blocked / shelf"),
        Patch(color="#42b883", label="Computed route"),
    ], loc="upper center", bbox_to_anchor=(0.5, -0.08), ncol=2, frameon=False)
    fig.tight_layout()
    st.pyplot(fig, use_container_width=True)
    plt.close(fig)

with right:
    st.subheader("A* result")
    if path:
        st.success("Route found — shortest path under equal move costs.")
        st.write(f"**Start:** {start}  \n**Destination:** {goal}")
        st.write(f"**Moves:** {len(path)-1}  \n**Cells in path:** {len(path)}")
        st.write("**Route coordinates (row, column):**")
        st.code(" → ".join(str(p) for p in path), language="text")
    else:
        st.error("No valid route exists with the current obstacles.")
        st.write("Clear or move some shelves, then A* will search again.")

    st.divider()
    st.subheader("Dynamic re-routing demo")
    st.write("Add a new shelf directly onto the current route, then recalculate. The route is recomputed on the updated grid.")
    if path and st.button("Block route & re-route", type="primary", use_container_width=True):
        candidates = [p for p in path[1:-1] if p not in (start, goal)]
        if candidates:
            # Block a middle route cell; A* recomputes on the next rerun.
            cell = candidates[len(candidates)//2]
            st.session_state.blocked.add(cell)
            st.session_state.last_path = []
            st.rerun()
        else:
            st.info("No intermediate route cell to block.")
    if st.button("Compare A* vs BFS", use_container_width=True):
        from collections import deque
        t0 = time.perf_counter()
        q = deque([start]); prev = {start: None}; visited = 0
        while q:
            cur = q.popleft()
            if cur == goal:
                break
            visited += 1
            r,c = cur
            for nb in ((r-1,c),(r+1,c),(r,c-1),(r,c+1)):
                if 0 <= nb[0] < n and 0 <= nb[1] < n and nb not in blocked and nb not in prev:
                    prev[nb] = cur; q.append(nb)
        bfs_ms = (time.perf_counter()-t0)*1000
        st.write(f"**A\*:** {explored} nodes, {elapsed:.3f} ms")
        st.write(f"**BFS:** {visited} nodes, {bfs_ms:.3f} ms")
        st.caption("Times are local runtime measurements and can vary between runs.")

with st.expander("How the A* algorithm works"):
    st.markdown("""
    1. Start with `g(start) = 0`.
    2. Estimate remaining distance using Manhattan distance: `h = |r₁-r₂| + |c₁-c₂|`.
    3. Select the open cell with the lowest `f = g + h`.
    4. Check up, down, left and right neighbours; skip blocked or out-of-grid cells.
    5. Update the best cost and parent for improved routes.
    6. Stop when the destination is reached; reconstruct the path from parent links.

    With four-direction movement and equal move costs, Manhattan distance is admissible, so A* returns a shortest path when one exists.
    """)
st.caption("Prototype assumptions: one robot, four-direction movement, every move costs 1. Search-time values are measurements from this run, not fixed benchmarks.")
