import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, box
import scipy as sp

def perpendicular_line(line, bounding_box, length=4, from_beginning=True, direction=1):
    # Extract start and end points
    x1, y1, x2, y2 = *line.coords[0], *line.coords[1]
    
    # Compute direction vector of the line
    dx, dy = x2 - x1, y2 - y1
    
    # Compute perpendicular vector (rotated 90 degrees)
    perp_dx, perp_dy = -dy, dx  # 90-degree rotation
    
    # Normalize and scale the perpendicular vector
    magnitude = (perp_dx**2 + perp_dy**2)**0.5
    if direction == -1:
        magnitude = -magnitude

    perp_dx, perp_dy = (perp_dx / magnitude) * length, (perp_dy / magnitude) * length
    
    # Choose a base point (midpoint or an endpoint)
    if from_beginning:
        base_x, base_y = x1, y1
    else:
        base_x, base_y = x2, y2  # Use the second endpoint
    
    # Compute perpendicular line endpoints
    perp_x1, perp_y1 = base_x, base_y
    perp_x2, perp_y2 = base_x + perp_dx, base_y + perp_dy

    original_line = LineString([(perp_x1, perp_y1), (perp_x2, perp_y2)])

    return original_line.intersection(bounding_box)

def exteriorConvexHull(hull, hull_vertices, minx, miny, maxx, maxy):
    hull = np.array(hull + [hull[0]])
    hull_vertices = (hull_vertices + [hull_vertices[0]])
    hull_vertices_new = []
    
    bbox = box(minx, miny, maxx, maxy)
    bbox_array = [[maxx, maxy], [minx, maxy], [maxx, miny], [minx, miny]]
    
    perp_line = np.empty((0, 4))
    resultingHull = []

    for point in range(len(hull) - 1):
        line = LineString([hull[point], hull[point + 1]])
        perp_line_front = perpendicular_line(line, bbox, direction=-1)

        # direction = -1 means the line will always extend away from the convex hull
        perp_line_end = perpendicular_line(line, bbox, from_beginning=False, direction=-1)

        x, y = line.xy
        px1, py1 = perp_line_front.xy
        px2, py2 = perp_line_end.xy

        perp_line = np.vstack((perp_line, [px1[0], py1[0], px1[1], py1[1]]))
        perp_line = np.vstack((perp_line, [px2[0], py2[0], px2[1], py2[1]]))

        hull_vertices_new = hull_vertices_new + [hull_vertices[point]]
        hull_vertices_new = hull_vertices_new + [hull_vertices[point + 1]]

    perp_line = np.vstack((perp_line, perp_line[0]))
    hull_vertices_new = hull_vertices_new + [hull_vertices_new[0]]
    
    for i in range(len(perp_line) - 1):
        currentHullPoint = [
            [perp_line[i][0], perp_line[i][1]], [perp_line[i][2], perp_line[i][3]], 
            [perp_line[i+1][0], perp_line[i+1][1]], [perp_line[i+1][2], perp_line[i+1][3]]
        ]

        line1 = LineString([[perp_line[i][2], perp_line[i][3]], [perp_line[i+1][2], perp_line[i+1][3]]])
        for boxPoint in range(len(bbox_array)):
            line2 = LineString([[perp_line[i][0], perp_line[i][1]], [bbox_array[boxPoint][0], bbox_array[boxPoint][1]]])
            if line1.intersects(line2):
                currentHullPoint = currentHullPoint + [[bbox_array[boxPoint][0], bbox_array[boxPoint][1]]]
               
        convexType = "rect"
        originalHullIntersect = [currentHullPoint[0], currentHullPoint[2]]
        originalHullIntersectIndex = [hull_vertices_new[i], hull_vertices_new[i+1]]
        if (np.allclose(currentHullPoint[0], currentHullPoint[2], atol=1e-4)):
            convexType = "tri"
            originalHullIntersect = [currentHullPoint[0]]
            originalHullIntersectIndex = [hull_vertices_new[i]]

        region = sp.spatial.Delaunay(currentHullPoint)
        currentHullPoint = np.array(currentHullPoint)
        resultingHull.append([region, convexType, originalHullIntersect, originalHullIntersectIndex])

    return resultingHull

