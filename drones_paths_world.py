#!/usr/bin/env python

import sys, time, math
import matplotlib.pyplot as plt
import shapely.geometry, shapely.ops, networkx
import pandas as pd, numpy as np

class World:
    def __init__(self, frame, walls, centers, nodes, route_edges, route):
        self.frame = frame
        self.walls = walls
        self.centers = centers
        self.nodes = nodes
        self.route_edges = route_edges
        self.route = route

    def split_route_to_k(self, k):
        if k > len(self.route):
            raise Exception('the number of drones is larger then the number of cells')
        # route = self.route.copy()
        route = list(self.route)
        n = len(route) // k
        routes = []
        for i in range(k):
            routes.append(route)
            route = route[n:] + route[:n]
        return routes

    @staticmethod
    def add_h_t_to_routes(routes, h, dt):
        routes_ht = []
        for route in routes:
            routes_ht.append([(x,y,h,ix*dt) for ix,(x,y) in enumerate(route)])
        return routes_ht

    def get_k_routes_ht(self, k, h, dt):
        routes = self.split_route_to_k(k)
        routes_ht = self.add_h_t_to_routes(routes, h, dt)
        return routes_ht

    def visualize(self, ax):
        for geom in self.frame:
            xs, ys = geom.xy
            ax.plot(xs, ys, 'k', lw=1)
        for geom in self.walls:
            xs, ys = geom.xy
            ax.plot(xs, ys, 'k', lw=1)
        for (x0,y0),(x1,y1) in self.route_edges:
            ax.plot((x0,x1),(y0,y1), 'r', lw=1)

        xs = [p[0] for p in self.centers]
        ys = [p[1] for p in self.centers]
        ax.plot(xs, ys, 'bo')
        xs = [p[0] for p in self.nodes]
        ys = [p[1] for p in self.nodes]
        ax.plot(xs, ys, 'h', ms=10, color=(0.8,1,0))



def create_neighbors_diag(center, distance):
    tr = (center[0] + distance, center[1] + distance)
    br = (center[0] + distance, center[1] - distance)
    tl = (center[0] - distance, center[1] + distance)
    bl = (center[0] - distance, center[1] - distance)
    return [tl, bl, tr, br]

def surrounding_edges_for_horizontal_edge(edge, half_cell_size):
    se = []
    left_neighbors = create_neighbors_diag(edge[0], half_cell_size)
    right_neighbors = create_neighbors_diag(edge[1], half_cell_size)
    se.append((left_neighbors[0], left_neighbors[2]))
    se.append((left_neighbors[1], left_neighbors[3]))
    se.append((left_neighbors[1], left_neighbors[0]))
    se.append((right_neighbors[0], right_neighbors[2]))
    se.append((right_neighbors[1], right_neighbors[3]))
    se.append((right_neighbors[3], right_neighbors[2]))
    se.append((left_neighbors[2], right_neighbors[0]))
    se.append((left_neighbors[3], right_neighbors[1]))
    return se

def surrounding_edges_for_vertical_edge(edge, half_cell_size):
    se = []
    bottom_neighbors = create_neighbors_diag(edge[0], half_cell_size)
    top_neighbors = create_neighbors_diag(edge[1], half_cell_size)
    se.append((bottom_neighbors[1], bottom_neighbors[0]))
    se.append((bottom_neighbors[3], bottom_neighbors[2]))
    se.append((bottom_neighbors[1], bottom_neighbors[3]))
    se.append((top_neighbors[1], top_neighbors[0]))
    se.append((top_neighbors[3], top_neighbors[2]))
    se.append((top_neighbors[0], top_neighbors[2]))
    se.append((bottom_neighbors[0], top_neighbors[1]))
    se.append((bottom_neighbors[2], top_neighbors[3]))
    return se


def create_world(requested_bounds, diameter):
    cell_size = math.sqrt((diameter**2)/2)
    half_cell_size = cell_size / 2
    double_cell_size = cell_size * 2

    minx = requested_bounds.bounds[0]
    miny = requested_bounds.bounds[1]
    maxx = requested_bounds.bounds[2]
    maxy = requested_bounds.bounds[3]
    width = maxx - minx
    height = maxy - miny

    num_cell_width = width / cell_size
    num_cell_height = height / cell_size
    num_double_cell_width = width / double_cell_size
    num_double_cell_height = height / double_cell_size

    wall_top = shapely.geometry.LineString(((minx,maxy),(maxx,maxy)))
    wall_left = shapely.geometry.LineString(((minx,miny),(minx,maxy)))
    wall_right = shapely.geometry.LineString(((maxx,miny),(maxx,maxy)))
    wall_bottom = shapely.geometry.LineString(((minx,miny),(maxx,miny)))
    frame = shapely.geometry.MultiLineString([wall_top, wall_left, wall_right, wall_bottom])

    centers = []
    for i in range(int(num_cell_width)):
        for j in range(int(num_cell_height)):
            center = (minx + half_cell_size + i * cell_size, miny + half_cell_size + j * cell_size)
            if requested_bounds.contains(shapely.geometry.Point(center)):
                centers.append(center)


    nodes = []
    for i in range(int(num_double_cell_width)):
        for j in range(int(num_double_cell_height)):
            node = (minx + cell_size + i * double_cell_size, miny + cell_size + j * double_cell_size)
            if requested_bounds.contains(shapely.geometry.Point(node)):
                nodes.append(node)

    g = networkx.Graph()
    for s in nodes:
        t1 = (s[0] + double_cell_size, s[1])
        t2 = (s[0] - double_cell_size, s[1])
        t3 = (s[0], s[1] + double_cell_size)
        t4 = (s[0], s[1] - double_cell_size)
        for t in [t1, t2, t3, t4]:
            if t in nodes:
                g.add_edge(s, t)


    t = networkx.minimum_spanning_tree(g, algorithm='kruskal')
    # t = networkx.minimum_spanning_tree(g, algorithm='prim')

    edges = [sorted(edge) for edge in t.edges]
    vertical_edges = []
    horizontal_edges = []
    for edge in edges:
        if edge[0][0] == edge[1][0]:
            vertical_edges.append(edge)
        elif edge[0][1] == edge[1][1]:
            horizontal_edges.append(edge)
        else:
            raise Exception(edge)

    vertical_edges = sorted(vertical_edges, key=lambda edge: edge[0])
    horizontal_edges = sorted(horizontal_edges, key=lambda edge: tuple(reversed(edge[0])))

    walls = shapely.geometry.MultiLineString(vertical_edges + horizontal_edges)

    surrounding_edges = set()
    for edge in horizontal_edges:
        surrounding_edges.update(surrounding_edges_for_horizontal_edge(edge, half_cell_size))
    for edge in vertical_edges:
        surrounding_edges.update(surrounding_edges_for_vertical_edge(edge, half_cell_size))
    route_edges = set()
    for se in surrounding_edges:
        line = shapely.geometry.LineString(se)
        if not line.crosses(walls):
            route_edges.add(se)

    g = networkx.Graph()
    g.add_edges_from(route_edges)
    node = list(g.nodes)[0]
    neighbors  = list(g.adj[node])
    g.remove_node(node)
    route = networkx.shortest_path(g, neighbors[0], neighbors[1])
    route.append(node) # and now the route is a circle
    return World(frame, walls, centers, nodes, route_edges, route)

def get_splitter(polygon):
    minx = polygon.bounds[0]
    miny = polygon.bounds[1]
    maxx = polygon.bounds[2]
    maxy = polygon.bounds[3]

    width = maxx - minx
    height = maxy - miny

    mx = (minx+maxx)/2
    my = (miny+maxy)/2

    vertical_splitter = shapely.geometry.LineString(((mx,miny),(mx,maxy)))
    horizontal_splitter = shapely.geometry.LineString(((minx,my),(maxx,my)))

    if width < height:
        return horizontal_splitter
    else:
        return vertical_splitter

def cut_polygon_by_line(polygon, line):
    merged = shapely.ops.linemerge([polygon.boundary, line])
    borders = shapely.ops.unary_union(merged)
    polygons = shapely.ops.polygonize(borders)
    return list(polygons)

def split_poligon(polygon, n = None):
    if n is None:
        line = get_splitter(polygon)
        return cut_polygon_by_line(polygon, line)

    polygons = [polygon]
    while len(polygons) < n:
        polygons.extend(split_poligon(polygons.pop(0)))
    return polygons








def get_polygon():
    polygon = shapely.geometry.Polygon([(0,30),(30,70),(70,80),(80,20),(60,0)])
    rhombus = shapely.geometry.Polygon([(-100,0),(0,100),(100,0),(0,-100),(-100,0)])
    square = shapely.geometry.Polygon([(-100,-100),(-100,100),(100,100),(100,-100),(-100,-100)])
    square_pwr_2 = shapely.geometry.Polygon([(-128,-128),(-128,128),(128,128),(128,-128),(-128,-128)])
    rectangle = shapely.geometry.Polygon([(-100,-50),(-100,50),(100,50),(100,-50),(-100,-50)])
    rectangle_narrow = shapely.geometry.Polygon([(0.5,0.5),(20.5,0.5),(20.5,2.5),(0.5,2.5),(0.5,0.5)])
    return square_pwr_2

# def test0():
#     fig, axs = plt.subplots()
#     axs.set_aspect('equal', 'datalim')
#
#     p = shapely.geometry.Polygon([(-20,-10),(-20,10),(20,10),(20,-10),(-20,-10)])
#     xs,ys = p.exterior.xy
#     axs.plot(xs, ys)
#
#     minx = p.bounds[0]
#     miny = p.bounds[1]
#     maxx = p.bounds[2]
#     maxy = p.bounds[3]
#     width = maxx - minx
#     height = maxy - miny
#
#     wall_top = shapely.geometry.LineString(((minx,maxy),(maxx,maxy)))
#     wall_left = shapely.geometry.LineString(((minx,miny),(minx,maxy)))
#     wall_right = shapely.geometry.LineString(((maxx,miny),(maxx,maxy)))
#     wall_bottom = shapely.geometry.LineString(((minx,miny),(maxx,miny)))
#
#     mx = (minx+maxx)/2
#     my = (miny+maxy)/2
#
#     print(minx, maxx, mx)
#     print(miny, maxy, my)
#
#     wall = shapely.geometry.LineString(((mx,miny),(mx,maxy)))
#     xs, ys = wall.xy
#     axs.fill(xs, ys, alpha=1, fc='r', ec='black')
#
#     plt.show()


def test1():

    diameter = 5*math.sqrt(2)

    figW, axW = plt.subplots()
    axW.set_aspect('equal', 'datalim')
    figS, axS = plt.subplots()
    axS.set_aspect('equal', 'datalim')

    requested_bounds = get_polygon()
    w = create_world(requested_bounds, diameter)
    w.visualize(axW)
    lst_r_w = w.get_k_routes_ht(k=2, h=-10,  dt=1)

    lst_r_s = []
    requested_bounds_lst = split_poligon(requested_bounds, 27)
    for bounds in requested_bounds_lst:
        w = create_world(bounds, diameter)
        w.visualize(axS)
        lst_r_s.append(w.get_k_routes_ht(k=1, h=-10, dt=1)[0])

    plt.show()

    for route in lst_r_w:
        print(route)
    for route in lst_r_s:
        print(route)

def test2():
    figW, axW = plt.subplots()
    axW.set_aspect('equal', 'datalim')

    df = pd.read_csv('markers_positions.csv')
    operations = []
    markers = []
    for ix,s in df.iterrows():
        color = (s.markerColorRed, s.markerColorGreen, s.markerColorBlue, s.markerColorOpacity)
        line = axW.plot(s.markerX, s.markerY, 's', color=color, ms=20)[0]
        line.set_visible(False)
        line.set_label(str(ix))
        markers.append(line)
        operations.append((s.markerStartSeconds, line, True))
        operations.append((s.markerStartSeconds+s.markerDurationSeconds, line, False))


    diameter = 5*math.sqrt(2)
    requested_bounds = shapely.geometry.Polygon([(-20,-10),(-20,10),(20,10),(20,-10),(-20,-10)])
    wW = create_world(requested_bounds, diameter)
    wW.visualize(axW)

    lst_r_W = wW.get_k_routes_ht(k=3, h=-10,  dt=1)
    x_data = np.zeros(len(lst_r_W))
    y_data = np.zeros_like(x_data)
    line = axW.plot(x_data, y_data, 'h', ms=20, color=(0,1,0,0.5))[0]
    line.set_visible(False)
    for ix in range(len(lst_r_W[0])):
        for jx in range(x_data.size):
            x,y,z,t = lst_r_W[jx][ix]
            x_data[jx] = x
            y_data[jx] = y
        operations.append((t, line, (x_data.copy(),y_data.copy())))

    t_prev = 0
    for ix in np.argsort([x[0] for x in operations]):
        t,line,args = operations[ix]
        if type(args) is bool:
            line.set_visible(args)
            print(f'{t}: Marker {line.get_label()} - {args}')
        else:
            x_data,y_data = args
            line.set_visible(True)
            line.set_xdata(x_data)
            line.set_ydata(y_data)
        plt.draw()
        for ix_m,marker in enumerate(markers):
            if not marker.get_visible(): continue
            xm = marker.get_xdata()[0]
            ym = marker.get_ydata()[0]
            r = np.sqrt((x_data-xm)**2 + (y_data-ym)**2)
            ix_min = np.argmin(r)
            if r[ix_min] < 5:
                print(f'{t} {ix_m} {ix_min} r_min={r[ix_min]:.3f}')
        dt = t - t_prev
        t_prev = t
        if dt>0: plt.pause(dt*0.3)
    plt.show()

if __name__ == '__main__':
    test1()
