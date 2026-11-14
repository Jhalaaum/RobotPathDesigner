"""Image-to-track graph. Coordinates are image pixels (x right, y down)."""
from dataclasses import dataclass
import math
import heapq
import numpy as np
from scipy import ndimage as ndi
from skimage.morphology import skeletonize

@dataclass(frozen=True)
class Node:
    id: int
    x: float
    y: float

@dataclass(frozen=True)
class Edge:
    start: int
    end: int
    points: tuple
    length_mm: float

@dataclass
class NavigationGraph:
    nodes: dict
    edges: list
    pixels_per_mm: float

    def shortest_path(self, start, end):
        """Return ordered edge traversals (edge, reversed), or raise ValueError."""
        if start not in self.nodes or end not in self.nodes:
            raise ValueError('Unknown node')
        adj = {n: [] for n in self.nodes}
        for e in self.edges:
            adj[e.start].append((e.end, e, False))
            adj[e.end].append((e.start, e, True))
        costs = {start: 0.0}; prev = {}; queue = [(0.0, start)]
        while queue:
            cost, n = heapq.heappop(queue)
            if cost != costs[n]: continue
            if n == end:
                path = []
                while n != start:
                    n, e, reverse = prev[n]; path.append((e, reverse))
                return path[::-1]
            for dest, e, reverse in adj[n]:
                new = cost + e.length_mm
                if new < costs.get(dest, math.inf):
                    costs[dest] = new; prev[dest] = (n, e, reverse)
                    heapq.heappush(queue, (new, dest))
        raise ValueError('Nodes are disconnected')


def extract_graph(rgb, pixels_per_mm, min_width_mm=10, max_width_mm=60, min_component_mm=65):
    """Width-filter dark neutral strokes; skeletonize; compress pixel chains.

    Width thresholds are heuristic, not semantic recognition. Wide dark areas
    and narrow printed details are suppressed. No gap bridging is performed.
    """
    if not math.isfinite(pixels_per_mm) or pixels_per_mm <= 0:
        raise ValueError('pixels_per_mm must be positive')
    if not 0 < min_width_mm < max_width_mm or min_component_mm < 0:
        raise ValueError('Invalid track filter settings')
    rgb = np.asarray(rgb)
    if rgb.ndim != 3 or rgb.shape[2] < 3: raise ValueError('Expected RGB image')
    rgb = rgb[:, :, :3].astype(np.int16)
    hi, lo = rgb.max(axis=2), rgb.min(axis=2)
    mask = (hi < 90) & (hi-lo < 45)
    radius = ndi.distance_transform_edt(mask)
    # Remove bulky dark regions before thinning, while retaining junctions.
    broad = radius > max_width_mm * pixels_per_mm / 2
    if broad.any():
        mask &= ndi.distance_transform_edt(~broad) > max_width_mm * pixels_per_mm / 3
    skel = skeletonize(mask)
    skel &= radius >= min_width_mm * pixels_per_mm / 2
    labels, count = ndi.label(skel, np.ones((3,3)))
    sizes = np.bincount(labels.ravel()); keep = sizes >= min_component_mm*pixels_per_mm; keep[0] = False
    skel = keep[labels]
    graph = graph_from_skeleton(skel, pixels_per_mm)
    return split_corners(graph)


def graph_from_skeleton(skeleton, pixels_per_mm):
    """Compress degree-two chains, including isolated cycles and junction clusters."""
    pixels = {(int(x),int(y)) for y,x in np.argwhere(skeleton)}
    adj = {}
    for x,y in pixels:
        ns = []
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            q = (x+dx,y+dy)
            if q not in pixels: continue
            if dx and dy and ((x+dx,y) in pixels or (x,y+dy) in pixels): continue
            ns.append(q)
        adj[x,y] = sorted(ns)
    critical = {p for p in pixels if len(adj[p]) != 2}
    nodes = {}; owner = {}; clusters = []; todo = set(critical)
    def add_node(group):
        i = len(nodes); group = sorted(group); clusters.append(group)
        nodes[i] = Node(i, sum(p[0] for p in group)/len(group), sum(p[1] for p in group)/len(group))
        for p in group: owner[p] = i
    while todo:
        p = min(todo); todo.remove(p); group = [p]
        # Only merge adjacent branching pixels, never swallow short endpoints.
        for q in group:
            if len(adj[q]) < 3: continue
            for r in adj[q]:
                if r in todo and len(adj[r]) >= 3: todo.remove(r); group.append(r)
        add_node(group)
    # One anchor per pure cycle.
    seen = set();
    for p in sorted(pixels):
        if p in seen: continue
        stack=[p]; seen.add(p); component=[]
        while stack:
            q=stack.pop(); component.append(q)
            for r in adj[q]:
                if r not in seen: seen.add(r); stack.append(r)
        if not any(q in owner for q in component): add_node([min(component)])
    visited=set(); edges=[]
    def key(a,b): return tuple(sorted((a,b)))
    for i, group in enumerate(clusters):
        for p in group:
            for q in adj[p]:
                if owner.get(q)==i or key(p,q) in visited: continue
                chain=[p,q]; visited.add(key(p,q)); a,b=p,q
                while b not in owner:
                    nxt=next(r for r in adj[b] if r!=a)
                    visited.add(key(b,nxt)); chain.append(nxt); a,b=b,nxt
                j=owner[b]; start=nodes[i]; end=nodes[j]
                points=((start.x,start.y), *chain, (end.x,end.y))
                length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))/pixels_per_mm
                if length: edges.append(Edge(i,j,points,length))
    return NavigationGraph(nodes,edges,pixels_per_mm)


def split_corners(graph, tolerance_mm=5, minimum_angle=35):
    """Add nodes at significant bends; retain original pixel geometry and length."""
    nodes=dict(graph.nodes); edges=[]
    def simplify(points, a, b):
        if b-a < 2: return [a,b]
        u=np.asarray(points[a]); v=np.asarray(points[b]); d=v-u
        middle=np.asarray(points[a+1:b]); denom=float(d@d)
        t=np.clip((middle-u)@d/denom,0,1) if denom else np.zeros(len(middle))
        distances=np.linalg.norm(middle-u-t[:,None]*d,axis=1)
        j=int(np.argmax(distances))+a+1
        if distances[j-a-1] <= tolerance_mm*graph.pixels_per_mm: return [a,b]
        return simplify(points,a,j)[:-1]+simplify(points,j,b)
    for edge in graph.edges:
        points=edge.points; idx=simplify(points,0,len(points)-1); cuts=[0]
        for a,b,c in zip(idx,idx[1:],idx[2:]):
            u=np.asarray(points[b])-points[a]; v=np.asarray(points[c])-points[b]
            norm=np.linalg.norm(u)*np.linalg.norm(v)
            if norm and math.degrees(math.acos(float(np.clip(u@v/norm,-1,1)))) >= minimum_angle: cuts.append(b)
        cuts.append(len(points)-1); start=edge.start
        for a,b in zip(cuts,cuts[1:]):
            end=edge.end if b==len(points)-1 else len(nodes)
            if end not in nodes: nodes[end]=Node(end,*points[b])
            part=points[a:b+1]; length=sum(math.dist(u,v) for u,v in zip(part,part[1:]))/graph.pixels_per_mm
            if length: edges.append(Edge(start,end,part,length))
            start=end
    return NavigationGraph(nodes,edges,graph.pixels_per_mm)
