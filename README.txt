WAREHOUSE ROBOT ROUTE OPTIMIZER — A* SIMULATOR

Files:
- warehouse_robot_simulator.py : Streamlit simulator
- requirements_warehouse_simulator.txt : required Python packages

Run on Windows:
1. Install Python 3.10+ if needed.
2. Open Command Prompt in the folder containing these files.
3. Run:
   py -m venv .venv
   .venv\Scripts\activate
   py -m pip install -r requirements_warehouse_simulator.txt
   streamlit run warehouse_robot_simulator.py
4. The simulator opens in your browser (usually http://localhost:8501).

Features:
- Adjustable grid size and obstacle density
- Generate random warehouse layouts with a repeatable seed
- A* pathfinding using Manhattan distance and four-direction moves
- Path length, explored nodes, and measured search time
- No-route detection
- Dynamic obstacle insertion and re-routing demo
- A* vs BFS comparison
- Route coordinates and visual legend

Notes:
- Search-time measurements depend on your machine and the current layout.
- Assumptions: one robot, equal movement cost, no diagonal movement.
