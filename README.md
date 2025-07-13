🤖 6-DOF Robot PRM Planner

This project implements a Probabilistic Roadmap (PRM) for a 6-joint robot arm.
It generates random joint configurations, checks collisions via a Godot simulator, builds a roadmap of safe nodes and edges, and finds paths with A*.

⚙️ Requirements

    Python 3.7+

    Godot Engine (>= 3.5)

    Python packages:

    pip install numpy networkx scipy tqdm

🚦 How it works

    Nodes: Random safe configurations are generated and stored.

    Collision check: Each config is sent via UDP to Godot, which checks for self-collision or obstacles.

    Edges: Pairs of nodes are connected if the interpolated path is safe.

    Graph: Built with networkx and used for path planning.

📡 Godot Simulator

You must run a Godot simulation that:

    Listens on UDP (SERVER_IP, SERVER_PORT).

    Receives joint configurations in JSON.

    Simulates the robot and checks collisions.

    Returns 1 (unsafe) or 0 (safe).

▶️ Run

Start the Godot simulator, then:

python main.py

📂 Files

    nodes.txt — Safe configs.

    arcs.txt — Safe edges.
    
Need help with the Godot side? Just ask! 🚀
