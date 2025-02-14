import numpy as np
import matplotlib.pyplot as pl
import scipy as sp
import sys
from shapely.geometry import Point, LineString
from exConvex import exteriorConvexHull
from scipy.ndimage import gaussian_filter
from scipy.ndimage import map_coordinates
import lic

# eps = sys.float_info.epsilon
def bernstein(n, i, t):
    return sp.special.comb(n, i) * (t ** i) * ((1 - t) ** (n - i))

def bezier_curve(t, P):
    """Compute a cubic Bézier curve given control points P."""
    return (1 - t)**3 * P[0] + 3 * (1 - t)**2 * t * P[1] + 3 * (1 - t) * t**2 * P[2] + t**3 * P[3]

def in_box(towers, bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                         towers[:, 0] <= bounding_box[1]),
                          np.logical_and(bounding_box[2] <= towers[:, 1],
                                         towers[:, 1] <= bounding_box[3]))

def points_equal(p1, p2, tolerance=1e-4):
    return np.allclose(p1, p2, atol=tolerance)

# Function to find the third vertex of a triangle containing P1 and P2
def find_third_vertex(simplices, points, P1, P2, tolerance=1e-4):
    for simplex in simplices:
        # Extract the triangle vertices
        triangle_vertices = [points[simplex[0]], points[simplex[1]], points[simplex[2]]]
        
        # Check if P1 and P2 are in the triangle (with tolerance)
        if any(points_equal(P1, vertex, tolerance) for vertex in triangle_vertices) and \
           any(points_equal(P2, vertex, tolerance) for vertex in triangle_vertices):
            # Find the third vertex of the triangle
            third_vertex = [vertex for vertex in triangle_vertices if not (points_equal(P1, vertex, tolerance) or points_equal(P2, vertex, tolerance))]
            return third_vertex[0]
    return None

def barycentric_coordinates(triangle, point):
    # Unpack the triangle vertices (A, B, C) and the point (P)
    A, B, C = triangle
    P = point

    # Calculate the area of the triangle ABC
    area_total = abs(A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1])) / 2

    # Calculate the area of sub-triangle PBC
    area_PBC = abs(P[0] * (B[1] - C[1]) + B[0] * (C[1] - P[1]) + C[0] * (P[1] - B[1])) / 2

    # Calculate the area of sub-triangle PCA
    area_PCA = abs(A[0] * (P[1] - C[1]) + P[0] * (C[1] - A[1]) + C[0] * (A[1] - P[1])) / 2

    # Calculate the area of sub-triangle PAB
    area_PAB = abs(A[0] * (B[1] - P[1]) + B[0] * (P[1] - A[1]) + P[0] * (A[1] - B[1])) / 2

    # Calculate the barycentric coordinates
    lambda_1 = area_PBC / area_total
    lambda_2 = area_PCA / area_total
    lambda_3 = area_PAB / area_total

    return (lambda_1, lambda_2, lambda_3)

def voronoi(towers, bounding_box):
    # Select towers inside the bounding box
    i = in_box(towers, bounding_box)
    # Mirror points
    points_center = towers[i, :]
    points_left = np.copy(points_center)
    points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
    points_right = np.copy(points_center)
    points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
    points_down = np.copy(points_center)
    points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
    points_up = np.copy(points_center)
    points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
    points = np.append(points_center, np.append(np.append(points_left, points_right, axis=0), np.append(points_down, points_up, axis=0), axis=0), axis=0)
    # Compute Voronoi
    vor = sp.spatial.Voronoi(points)
    # Filter regions
    regions = []
    for region in vor.regions:
        flag = True
        for index in region:
            if index == -1:
                flag = False
                break
            else:
                x = vor.vertices[index, 0]
                y = vor.vertices[index, 1]
                eps = sys.float_info.epsilon

                if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                       bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                    flag = False
                    break
        if region != [] and flag:
            regions.append(region)
    vor.filtered_points = points_center
    vor.filtered_regions = regions
    return vor

def centroid_region(vertices):
    # Polygon's signed area
    A = 0
    # Centroid's x
    C_x = 0
    # Centroid's y
    C_y = 0
    for i in range(0, len(vertices) - 1):
        s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
        A = A + s
        C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
        C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
    A = 0.5 * A
    C_x = (1.0 / (6.0 * A)) * C_x
    C_y = (1.0 / (6.0 * A)) * C_y
    return np.array([[C_x, C_y]])

def calculate_first_slope(bezier_order, control_point, bezier_point):
    sumBernstein = 0
    for i in range(bezier_order - 1):
        product = np.outer(bernstein(bezier_order - 1, i, bezier_point), control_point[i + 1] - control_point[i])
        sumBernstein = sumBernstein + product
    
    return sumBernstein * bezier_order

def calculate_second_slope(bezier_order, control_point, bezier_point):
    sumBernstein = 0
    for i in range(bezier_order - 2):
        product = np.outer(bernstein(bezier_order - 2, i, bezier_point), control_point[i + 2] - 2 * control_point[i + 1] + control_point[i])
        sumBernstein = sumBernstein + product

    return bezier_order * (bezier_order - 1) * sumBernstein

def line_integral_convolution(vector_field, noise_texture, kernel_length=20, steps=10):
    """
    Perform Line Integral Convolution (LIC) on a 2D vector field.

    Parameters:
    - vector_field: 2D array of shape (height, width, 2) representing the vector field (u, v).
    - noise_texture: 2D array of shape (height, width) representing the noise texture.
    - kernel_length: Length of the convolution kernel (default: 20).
    - steps: Number of steps to integrate along the vector field (default: 10).

    Returns:
    - lic_image: 2D array of shape (height, width) representing the LIC image.
    """
    height, width, _ = vector_field.shape
    lic_image = np.zeros_like(noise_texture)

    # Normalize the vector field
    magnitude = np.sqrt(vector_field[:, :, 0]**2 + vector_field[:, :, 1]**2)
    vector_field[:, :, 0] /= magnitude + 1e-10
    vector_field[:, :, 1] /= magnitude + 1e-10

    # Iterate over each pixel
    for y in range(height):
        for x in range(width):
            # Initialize the integral curve
            curve = []
            curve.append((y, x))

            # Integrate forward
            current_y, current_x = y, x
            for _ in range(steps):
                u, v = vector_field[int(current_y), int(current_x)]
                current_y += v
                current_x += u
                if current_y < 0 or current_y >= height or current_x < 0 or current_x >= width:
                    break
                curve.append((current_y, current_x))

            # Integrate backward
            current_y, current_x = y, x
            for _ in range(steps):
                u, v = vector_field[int(current_y), int(current_x)]
                current_y -= v
                current_x -= u
                if current_y < 0 or current_y >= height or current_x < 0 or current_x >= width:
                    break
                curve.insert(0, (current_y, current_x))

            # Sample the noise texture along the curve
            curve = np.array(curve)
            sampled_values = map_coordinates(noise_texture, curve.T, order=1)

            # Convolve with a kernel (e.g., a box filter)
            kernel = np.ones(kernel_length) / kernel_length
            convolved_value = np.convolve(sampled_values, kernel, mode='valid')

            # Assign the result to the LIC image
            lic_image[y, x] = convolved_value[len(convolved_value) // 2]

    return lic_image


fig = pl.figure()

ax = fig.gca()

np.set_printoptions(precision=3, suppress=True)  # Set printing precision
ax.set_xlim([-1.5, 1.5])
ax.set_ylim([-1.5, 1.5])

# Define grid size
N = 100
x = np.linspace(-1, 1, N)
y = np.linspace(-1, 1, N)
X, Y = np.meshgrid(x, y)

#IMPORTANT, this number changes everything
gradeBezier = 4

# Control points for an S-shaped cubic Bézier curve
P = np.array([[-0.8, -0.8], [-0.4, -0.8], [0.2, 1], [0.8, 0.8]])

# Generate Bézier curve points
T = np.linspace(0, 1, 18)
bezier_points = np.array([bezier_curve(t, P) for t in T])

# Create Delaunay triangulation using scipy's Delaunay
delaunay = sp.spatial.Delaunay(bezier_points)

# Compute convex hull
hull = sp.spatial.ConvexHull(bezier_points)

hull_vertices = np.round(bezier_points[hull.vertices], 3)

outerHull = exteriorConvexHull(hull_vertices.tolist(), -1, -1, 1, 1)

delaunayHull = sp.spatial.Delaunay(hull_vertices)

grid_points = np.vstack([X.ravel(), Y.ravel()]).T

query_point = np.array([-0.25, -0.25])

bounding_box = np.array([-1, 1, -1, 1]) # [x_min, x_max, y_min, y_max]

vor = voronoi(bezier_points, bounding_box)

area = []

normalisation_const = 5

# Calculating the area for each polygon in the Voronoi diagram
for i in range(len(vor.filtered_regions)):
    # ax.plot(bezier_points[i, 0], bezier_points[i, 1], 'b.')
    # pl.draw()
    # pl.pause(0.5)

    regionIndex = vor.point_region[i]
    region = vor.regions[regionIndex]
    vertices = vor.vertices[region + [region[0]], :]

    area = area +  [sp.spatial.ConvexHull(vor.vertices[region, :]).volume]

    # ax.plot(vertices[:, 0], vertices[:, 1], 'k-')

    # pl.draw()
    # pl.pause(0.5)
slopeListX = []
slopeListY = []
failedVor = 0

for point in grid_points:
    if delaunay.find_simplex(point) != -1:

        bezier_points = np.vstack([bezier_points, point])
        vorUpdated = voronoi(bezier_points, bounding_box)

        areaUpdated = []
        areaPercentage = []
        # ax.cla()
        # ax.scatter(bezier_points[:, 0], bezier_points[:, 1], color='red', zorder=5)
        # pl.draw()

        for i in range(len(vorUpdated.filtered_regions)):
            regionIndex = vorUpdated.point_region[i]
            region = vorUpdated.regions[regionIndex]
            vertices = vorUpdated.vertices[region + [region[0]], :]
            areaUpdated = areaUpdated +  [sp.spatial.ConvexHull(vorUpdated.vertices[region, :]).volume]
            #ax.plot(vertices[:, 0], vertices[:, 1], 'k-')

        # print(len(area), len(areaUpdated))
        if len(areaUpdated) < len(T) + 1:
            # print(point)
            bezier_points = bezier_points[:-1]
            slopeListX = slopeListX + [1e-4]
            slopeListY = slopeListY + [1e-4]
            failedVor = failedVor + 1
            continue

        for i in range(len(area)):
            areaPercentage = areaPercentage + [abs(area[i] - areaUpdated[i]) / areaUpdated[-1]]

        slopeAtPoint = 0

        for i in range(len(areaPercentage)):
            slopeAtPoint = slopeAtPoint + (areaPercentage[i] * calculate_first_slope(gradeBezier, P, T[i]))
        
        bezier_points = bezier_points[:-1]
        # pl.pause(0.1)
        # print(slopeAtPoint[0])
        slopeListX = slopeListX + [slopeAtPoint[0][0]]
        slopeListY = slopeListY + [slopeAtPoint[0][1]]

    
    else:
        found = False
        for object in outerHull:
            if object[0].find_simplex(point) != -1:
                found = True
                xyOut = None
                if object[1] == "tri":
                    # print(object[2][0][0])
                    xyOut = calculate_first_slope(gradeBezier, P, object[2][0][0]) + calculate_second_slope(gradeBezier, P, object[2][0][0]) \
                          * (np.linalg.norm(point - object[2][0]) / normalisation_const) 
                    
                elif object[1] == "rect":
                    vertex3 = find_third_vertex(delaunayHull.simplices, hull_vertices, object[2][0], object[2][1])
                    line = LineString([object[2][0], object[2][1]])
                    projected_point = line.interpolate(line.project(Point(point[0], point[1])))
                    lambda_1, lambda_2, lambda_3 = barycentric_coordinates([vertex3, object[2][0], object[2][1]], [projected_point.x, projected_point.y])
                    firstInter = lambda_2 * (calculate_first_slope(gradeBezier, P, object[2][0][0]) + calculate_second_slope(gradeBezier, P, object[2][0][0]) \
                                            * (np.linalg.norm(point - [projected_point.x, projected_point.y]) / normalisation_const))
                    secondInter = lambda_3 * (calculate_first_slope(gradeBezier, P, object[2][1][0]) + calculate_second_slope(gradeBezier, P, object[2][1][0]) \
                                            * (np.linalg.norm(point - [projected_point.x, projected_point.y]) / normalisation_const))
                    xyOut = firstInter + secondInter

                slopeListX = slopeListX + [xyOut[0][0]]
                slopeListY = slopeListY + [xyOut[0][1]]
                break

        if not found:    
            print("cannot bound")
            slopeListX = slopeListX + [1e-4]
            slopeListY = slopeListY + [1e-4]
        

slopeArrX = np.array(slopeListX)
slopeArrY = np.array(slopeListY)

print(failedVor)
u = slopeArrX.reshape(100, 100)
v = slopeArrY.reshape(100, 100)

vector_field = np.stack((u, v), axis=-1)

lic_image = line_integral_convolution(vector_field, np.random.rand(100, 100))

pl.streamplot(X, Y, u, v, color='black', density=2)
pl.scatter(bezier_points[:, 0], bezier_points[:, 1], color='blue', s=50, label='Bézier Points')
pl.imsave("lic.png", lic_image, cmap='gray')

pl.show()

# Plot the LIC image

# xv, yv = np.meshgrid(slopeListX, slopeListY)
# for simplex in hull.simplices:
#     ax.plot(bezier_points[simplex, 0], bezier_points[simplex, 1], 'b-', label='Convex Hull' if simplex[0] == hull.simplices[0][0] else "")
        
# pl.streamplot(X, Y, u, v, color='black', density=2)
# pl.scatter(bezier_points[:, 0], bezier_points[:, 1], color='blue', s=50, label='Bézier Points')
# for simplex in hull.simplices:
#     pl.plot(bezier_points[simplex, 0], bezier_points[simplex, 1], 'g-', linewidth=2, label='Convex Hull' if 'Convex Hull' not in pl.gca().get_legend_handles_labels()[1] else "")

# for i in hull.vertices:
#     print(bezier_points[i])


pl.show()

