from itertools import product
import math
import random
import struct
import threading
import numpy as np
import networkx as nx
import time
from socket import *
import json
from tqdm import tqdm
from scipy.spatial import KDTree
from itertools import combinations

SERVER_IP = "localhost"
# SERVER_IP = "192.168.55.162"
SERVER_PORT = 4321

STEP = 1
sendTrue = False

clientSocket = socket(AF_INET, SOCK_DGRAM)
clientSocket.settimeout(1)

ROBOT_ROTATION_RANGE = {
    "r1": (-math.pi, math.pi),
    "r2": (-math.pi, math.pi),
    "r3": (-math.pi, math.pi),
    "r4": (-math.pi, math.pi),
    "r5": (-math.pi, math.pi),
    "r6": (-math.pi, math.pi),
}

ROBOT_ACTION_SPACE = {
    "xrange": (-1, 1),
    "yrange": (-1, 1),
    "zrange": (0, 1),
}

nodes = []  # 6 dim
arcs = []


def generate_ranges(ranges_dict, step):
    return [
        np.round(np.arange(start, end, step), 2) for start, end in ranges_dict.values()
    ]


def random_config():
    return tuple(
        round(random.uniform(start, end), 2)
        for start, end in ROBOT_ROTATION_RANGE.values()
    )


def generate_nodes_monte_carlo(N=1000):
    nodes = []
    with open("nodes.txt", "w") as myfile:
        with tqdm(total=N, desc="Generating Nodes") as pbar:
            while len(nodes) < N:
                config = random_config()
                if isSafe(config):
                    nodes.append(config)
                    line = "\n" + " ".join(f"{x:.2f}" for x in config)
                    myfile.write(line)
                    pbar.update(1)
    return nodes


def populate(nodes=[], full=False):
    if full:  # non stampa ancora su file, incompleta
        ranges = generate_ranges(ROBOT_ROTATION_RANGE, STEP)
        nodes.extends([tuple(coord) for coord in product(*ranges) if isSafe(coord)])
    else:
        nodes.extend(generate_nodes_monte_carlo())
    print(f"generated nodes: {len(nodes)}")
    return nodes


def isSafe(q) -> bool:
    global sendTrue, clientSocket
    message = {"q": [*q], "p": [*q], "T": 0, "profile": "capybara", "reset": sendTrue}

    addr = (SERVER_IP, SERVER_PORT)
    sendTrue = False
    try:
        clientSocket.sendto(json.dumps(message).encode("utf-8"), addr)
        data, addr = clientSocket.recvfrom(1)
    except socket.timeout:
        return False

    if data[0] == 1:
        sendTrue = True
        return False
    else:
        return True


def calcArcs(nodes, arcs=[], radius=1.5, steps=20):
    # print("Esempio nodo:", nodes[0])
    # print("Tipo:", type(nodes[0]))
    # print("Shape:", np.array(nodes).shape)
    nodes_arr = np.array(nodes)
    kdtree = KDTree(nodes_arr)
    with tqdm(total=len(nodes), desc="Generating Arcs") as pbar:
        with open("arcs.txt", "w") as arcsFile:
            total = len(nodes_arr)

            for i, node in enumerate(nodes_arr):
                indices = kdtree.query_ball_point(node, r=radius)

                for j in indices:
                    if i >= j:
                        continue  # se usiamo nx.Graph(), ogni arco (A,B) è automaticamente bidirezionale.

                    other = nodes_arr[j]

                    safe = True

                    for diff in np.linspace(0, 1, steps):
                        p = (1 - diff) * node + diff * other
                        if not isSafe(p):
                            safe = False
                            break

                    if safe:
                        arcs.append((nodes[i], nodes[j]))
                        line = (
                            " ".join(f"{x:.2f}" for x in nodes[i])
                            + " "
                            + " ".join(f"{x:.2f}" for x in nodes[j])
                            + "\n"
                        )
                        arcsFile.write(line)
                pbar.update(1)

    print(f"Total arcs: {len(arcs)}")
    print(f"Esempio arco: {arcs[0]}")
    return arcs


def isPathSafe(start, end, steps=20):
    """
    Controlla se il percorso lineare tra due configurazioni (start, end)
    è 'safe' usando steps punti di interpolazione.
    """
    for diff in np.linspace(0, 1, steps):
        p = (1 - diff) * np.array(start) + diff * np.array(end)
        if not isSafe(p):
            return False
    return True


def closestNodeToTarget(target, nodes):  # senza grafo
    target = np.array(target)  # (6,)
    nodes_arr = np.array(nodes)  # (N, 6)
    diffs = nodes_arr - target  # (N, 6)
    distances = np.linalg.norm(diffs, axis=1)  # (N,)

    idx = np.argmin(distances)
    return nodes[idx], distances[idx]


def findClosestNode(G, target):  # con grafo
    """
    Restituisce il nodo del grafo G più vicino alla tupla target.
    """
    target = np.array(target)
    min_dist = float("inf")
    closest = None

    for node in G.nodes:
        node_arr = np.array(node)
        dist = np.linalg.norm(node_arr - target)
        if dist < min_dist:
            min_dist = dist
            closest = node

    return closest, min_dist


def heuristic(n1, n2):
    return np.linalg.norm(np.array(n1) - np.array(n2))


def findPath(graph, start: tuple, end: tuple):
    if not isSafe(start) or not isSafe(end):
        print("Unreachable")
        return False
    closestStart = findClosestNode(graph, start)
    closestEnd = findClosestNode(graph, end)
    if not isPathSafe(start, closestStart) or not isPathSafe(end, closestEnd):
        print("Unreachable")
        return False
    try:
        path = nx.astar_path(
            graph, closestStart, closestEnd, heuristic=heuristic, weight="weight"
        )
    except nx.NetworkXNoPath:
        print("No path found!")
        return False

    path = [start] + path + [end]
    return path


def loadNodeFile(filename="nodes.txt"):
    nodes = []
    with open(filename, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            config = tuple(float(x) for x in parts)
            nodes.append(config)
    print(f"total nodes: {len(nodes)}")
    return nodes


def loadArcsFile(filename="arcs.txt"):
    arcs = []
    with open(filename, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 12:
                continue  # salta righe non valide
            node1 = tuple(float(x) for x in parts[:6])
            node2 = tuple(float(x) for x in parts[6:])
            arcs.append((node1, node2))
    print(f"Total arcs loaded: {len(arcs)}")
    return arcs


def buildGraph(nodes=None, arcs=None):
    """
    Se i nodi non vengono passati, networkx li ricava dagli archi. (meglio non passarli ovviamnt)
    """
    G = nx.Graph()

    if nodes:
        G.add_nodes_from(nodes)

    if arcs:
        for n1, n2 in arcs:
            dist = np.linalg.norm(np.array(n1) - np.array(n2))
            G.add_edge(n1, n2, weight=dist)

    print(f"Grafo creato con {G.number_of_nodes()} nodi e {G.number_of_edges()} archi.")
    return G


def testGraph(G):
    print(f"Nodes: {G.number_of_nodes()}")  # nodi
    print(f"Edges: {G.number_of_edges()}")  # archi
    degrees = [deg for _, deg in G.degree()]
    print(
        f"Degree min: {min(degrees)}, max: {max(degrees)}, avg: {sum(degrees)/len(degrees):.2f}"
    )  # Grado medio dei nodi, Grado massimo e minimo
    # Controlla connessività
    # Un grafo PRM buono di solito ha una grossa componente connessa, non mille isole minuscole.
    components = list(nx.connected_components(G))
    print(f"Number of connected components: {len(components)}")
    print(f"Biggest component size: {max(len(c) for c in components)}")
    # Controlla se i pesi sono plausibili.
    weights = [d["weight"] for _, _, d in G.edges(data=True)]
    print(f"Min edge weight: {min(weights):.2f}")
    print(f"Max edge weight: {max(weights):.2f}")
    # Prendi 2 nodi a caso e cerca un path.
    n1, n2 = random.sample(list(G.nodes), 2)
    try:
        p = nx.shortest_path(G, n1, n2, weight="weight")
        print("Example path:", p)
    except nx.NetworkXNoPath:
        print("No path between", n1, n2)


def main():
    # isSafe((1,2,3,4,5,6))
    # isSafe((3,2,4,6,7,8))
    nodes = loadNodeFile()
    nodes.extend(populate())
    arcs = loadArcsFile()
    arcs.extend(calcArcs(nodes))
    G = buildGraph(arcs=arcs)
    testGraph(G)
    clientSocket.close()  # always close the socket


if __name__ == "__main__":
    main()
