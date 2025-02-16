import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from scipy.special import comb
from Bezier import line_integral_convolution, vector_field

# Initialize figure
fig, ax = plt.subplots()
x = np.linspace(-1, 1, 100)
y = np.linspace(-1, 1, 100)
X, Y = np.meshgrid(x, y)
ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)
ax.set_title("Draw Freehand Lines, RDP Simplification and Bézier Fitting")

# Store drawn lines
all_lines = []
current_line = None
drawing = False

def perpendicular_distance(point, line_start, line_end):
    """Calculate perpendicular distance from a point to a line segment."""
    if np.all(line_start == line_end):
        return np.linalg.norm(point - line_start)
    
    line_vec = line_end - line_start
    point_vec = point - line_start
    line_len = np.dot(line_vec, line_vec)
    t = max(0, min(1, np.dot(point_vec, line_vec) / line_len))
    projection = line_start + t * line_vec
    return np.linalg.norm(point - projection)

def rdp(points, epsilon):
    """Ramer-Douglas-Peucker line simplification algorithm."""
    if len(points) < 3:
        return points
    
    # Find point with max distance from the line
    line_start, line_end = points[0], points[-1]
    distances = np.array([perpendicular_distance(p, line_start, line_end) for p in points])
    max_index = np.argmax(distances)
    max_distance = distances[max_index]

    if max_distance > epsilon:
        # Recursive RDP split
        left = rdp(points[:max_index+1], epsilon)
        right = rdp(points[max_index:], epsilon)
        return np.vstack((left[:-1], right))  # Merge without duplicate mid-point
    else:
        return np.array([line_start, line_end])

def bernstein_matrix(n, t):
    """Compute Bernstein basis matrix."""
    A = np.zeros((len(t), n + 1))
    for k in range(n + 1):
        A[:, k] = comb(n, k) * (1 - t) ** (n - k) * t ** k
    return A

def fit_bezier(x, y, n):
    """Fit a Bézier curve of degree n using least squares."""
    t = np.linspace(0, 1, len(x))
    A = bernstein_matrix(n, t)
    Px = np.linalg.lstsq(A, x, rcond=None)[0]
    Py = np.linalg.lstsq(A, y, rcond=None)[0]
    return Px, Py

def bezier_curve(Px, Py, num_points=300):
    """Compute Bézier curve points from control points."""
    n = len(Px) - 1
    t = np.linspace(0, 1-(1e-8), num_points)
    A = bernstein_matrix(n, t)
    bx = A @ Px
    by = A @ Py
    return bx, by

def on_press(event):
    """Start drawing a new line."""
    global drawing, current_line
    if event.inaxes:
        drawing = True
        current_line, = ax.plot([event.xdata], [event.ydata], 'r-', linewidth=2)
        all_lines.append(([], []))  # Store new empty line

def on_motion(event):
    """Update the current line as the mouse moves."""
    if drawing and event.inaxes:
        xdata, ydata = current_line.get_data()
        xdata = list(xdata) + [event.xdata]
        ydata = list(ydata) + [event.ydata]
        current_line.set_data(xdata, ydata)
        all_lines[-1] = (xdata, ydata)  # Store the updated points
        fig.canvas.draw()

def on_release(event):
    """Stop drawing when the mouse is released."""
    global drawing
    drawing = False

fig2, ax2 = plt.subplots(figsize=(8, 8))  # New figure for streamplot
ax2.set_title("Vector Field Streamplot")
ax2.set_xlabel("X-axis")
ax2.set_ylabel("Y-axis")


def simplify_and_update():
    """Apply RDP to simplify drawn lines, then fit Bézier curve and update canvas."""
    if not all_lines:
        return

    ax.cla()
    ax2.cla()
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_title("Fitted Bézier Curves after RDP Simplification")

    final_field = np.zeros((100, 100, 2))  
    for x, y in all_lines:
        if len(x) < 4:
            continue  # Skip very short lines

        # Apply RDP simplification
        points = np.column_stack((x, y))
        simplified = rdp(points, epsilon=0.1)
        sx, sy = simplified[:, 0], simplified[:, 1]

        # Fit a Bézier curve to the simplified line (max degree of 10)
        # degree = min(10, len(sx) - 1)
        degree = 3
        Px, Py = fit_bezier(sx, sy, degree)

        bx, by = bezier_curve(Px, Py)

        # Draw the Bézier curve
        ax.plot(bx, by, 'b-', linewidth=2)
        fig.canvas.draw()
        
        command = input("Proceed with vector generation? y/n")
        if command == 'n':
            all_lines.clear()
            ax.cla()
            return

        final_field = final_field + vector_field(np.vstack([Px, Py]).T, -1, 1, -1, 1)
        
    lic_image = line_integral_convolution(final_field, np.random.rand(100, 100))
    
    ax2.streamplot(X, Y, final_field[:, :, 0], final_field[:, :, 1], color='black', cmap='plasma', linewidth=1)
    
    ax.imshow(lic_image, cmap='gray', origin='lower', extent=[-1, 1, -1, 1])

    fig.canvas.draw()
    fig2.canvas.draw()
    all_lines.clear()  # Clear stored lines after fitting

def bezier_from_VF(field):
    
    return

# Connect event handlers
fig.canvas.mpl_connect("button_press_event", on_press)
fig.canvas.mpl_connect("motion_notify_event", on_motion)
fig.canvas.mpl_connect("button_release_event", on_release)

# Add button to simplify lines and fit Bézier
ax_simplify = plt.axes([0.8, 0.02, 0.1, 0.05])
btn_simplify = Button(ax_simplify, "Gen Vector Field")
btn_simplify.on_clicked(lambda event: simplify_and_update())

plt.show()
