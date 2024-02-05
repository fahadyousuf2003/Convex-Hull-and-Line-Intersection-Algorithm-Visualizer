# Requirements: Install tkinker to run this code
#               Use pip install tkinker to install the library

# Geometric Algorithms (Convex Hull and Line Intersection)

# Group Members :- 21K-4839 (Fahad Yousuf)
#                  21K-4838 (Huzaifa Asad)
#                  21K-4591 (Abdul Rafay)

import tkinter as tk
import random
import timeit
from functools import cmp_to_key

class Point:
    def __init__(self, x=None, y=None):
        self.x = x
        self.y = y

def orientation(p, q, r):
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    if val == 0:
        return 0  # collinear
    elif val > 0:
        return 1  # clockwise
    else:
        return 2  # counterclockwise

def onSegment(p, q, r):
    if (
        (q.x <= max(p.x, r.x))
        and (q.x >= min(p.x, r.x))
        and (q.y <= max(p.y, r.y))
        and (q.y >= min(p.y, r.y))
    ):
        return True
    return False

def calculate_det(a, b, c):
    return (a.x * b.y + b.x * c.y + c.x * a.y) - (a.y * b.x + b.y * c.x + c.y * a.x)

def dist_sq(p1, p2):
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2

def compare(p0, p1, p2):
    o = orientation(p0, p1, p2)
    if o == 0:
        if dist_sq(p0, p2) >= dist_sq(p0, p1):
            return -1
        else:
            return 1
    else:
        if o == 2:
            return -1
        else:
            return 1

def brute_force(points):
    if len(points) < 3:
        return []

    sorted_points = sorted(points, key=lambda p: (p.x, p.y))
    n = len(sorted_points)
    convex_set = set()

    p0 = min(sorted_points, key=lambda point: (point.y, point.x))

    for i in range(n - 1):
        for j in range(i + 1, n):
            points_left_of_ij = points_right_of_ij = True
            for k in range(n):
                if k != i and k != j:
                    det_k = calculate_det(
                        sorted_points[i], sorted_points[j], sorted_points[k]
                    )
                    if det_k > 0:
                        points_right_of_ij = False
                    elif det_k < 0:
                        points_left_of_ij = False
                    else:
                        if (
                            sorted_points[k].x < sorted_points[i].x
                            or sorted_points[k].x > sorted_points[j].x
                            or sorted_points[k].y < sorted_points[i].y
                            or sorted_points[k].y > sorted_points[j].y
                        ):
                            points_left_of_ij = points_right_of_ij = False
                            break

            if points_left_of_ij or points_right_of_ij:
                convex_set.update([sorted_points[i], sorted_points[j]])

    sorted_convex_set = sorted(convex_set, key=lambda p: (p.x, p.y))
    sorted_convex_set = sorted(
        sorted_convex_set, key=cmp_to_key(lambda p1, p2: compare(p0, p1, p2))
    )
    return sorted_convex_set

def jarvis_march(points):
    num_points = len(points)
    if num_points < 3:
        return []

    start_point = min(points, key=lambda p: (p.x, p.y))
    convex_hull_points = []

    current_point = start_point
    while True:
        convex_hull_points.append(current_point)
        next_point = None

        for candidate_point in points:
            if candidate_point != current_point:
                if (
                    next_point is None
                    or orientation(current_point, candidate_point, next_point) == 2
                ):
                    next_point = candidate_point

        current_point = next_point
        if current_point == start_point:
            break

    return convex_hull_points

def graham_scan(points):
    if len(points) < 3:
        return []

    p0 = min(points, key=lambda point: (point.y, point.x))
    candidate_points = sorted(points, key=cmp_to_key(lambda p1, p2: compare(p0, p1, p2)))

    stack = []
    stack.append(candidate_points[0])
    stack.append(candidate_points[1])
    stack.append(candidate_points[2])

    for i in range(3, len(candidate_points)):
        while (
            len(stack) > 1
            and orientation(stack[-2], stack[-1], candidate_points[i]) != 2
        ):
            stack.pop()
        stack.append(candidate_points[i])
    return stack

def quick_elimination_x_graham_scan(points):
    def isInsideBoundingBox(point, bounding_box):
        x_bb, y_bb = zip(
            *[(p.x, p.y) for p in bounding_box]
        )
        cross = 0
        for i in range(4):
            x1, y1, x2, y2 = x_bb[i - 1], y_bb[i - 1], x_bb[i], y_bb[i]

            if (x1 <= point.x < x2 or x2 <= point.x < x1) and point.y <= (
                (y2 - y1) / (x2 - x1)
            ) * (point.x - x1) + y1:
                cross += 1
        return cross % 2 != 0

    candidate_points = list(points)

    leftmost = min(candidate_points, key=lambda p: p.x)
    candidate_points.remove(leftmost)
    rightmost = max(candidate_points, key=lambda p: p.x)
    candidate_points.remove(rightmost)
    topmost = max(candidate_points, key=lambda p: p.y)
    candidate_points.remove(topmost)
    bottommost = min(candidate_points, key=lambda p: p.y)
    candidate_points.remove(bottommost)

    bounding_box = [topmost, leftmost, bottommost, rightmost]

    remaining_points = [
        point
        for point in candidate_points
        if not isInsideBoundingBox(point, bounding_box)
    ]
    for point in bounding_box:
        remaining_points.append(point)

    return graham_scan(remaining_points)

def monotone_chain(points):
    if len(points) < 3:
        return []

    candidate_points = sorted(set(points), key=lambda p: (p.x, p.y))

    lower = []
    for point in candidate_points:
        while len(lower) >= 2 and calculate_det(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)

    upper = []
    for point in reversed(candidate_points):
        while len(upper) >= 2 and calculate_det(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)

    convex_hull = lower[:-1] + upper[:-1]
    return convex_hull

def doIntersect(p1, q1, p2, q2):
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    if o1 != o2 and o3 != o4:
        return True

    if o1 == 0 and onSegment(p1, p2, q1):
        return True
    if o2 == 0 and onSegment(p1, q2, q1):
        return True
    if o3 == 0 and onSegment(p2, p1, q2):
        return True
    if o4 == 0 and onSegment(p2, q1, q2):
        return True

    return False

def intersect_slope_intercept(line1, line2):
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2

    # Calculate slopes
    slope1 = (y2 - y1) / (x2 - x1) if x2 - x1 != 0 else float('inf')
    slope2 = (y4 - y3) / (x4 - x3) if x4 - x3 != 0 else float('inf')

    # Calculate intersection point
    x_intersect = ((y3 - y1) - slope2 * x3 + slope1 * x1) / (slope1 - slope2) if slope1 != slope2 else None
    y_intersect = y1 + slope1 * (x_intersect - x1) if x_intersect is not None else None

    # Check if intersection point lies within line segments
    is_intersecting = (
        min(x1, x2) <= x_intersect <= max(x1, x2) and
        min(y1, y2) <= y_intersect <= max(y1, y2) and
        min(x3, x4) <= x_intersect <= max(x3, x4) and
        min(y3, y4) <= y_intersect <= max(y3, y4)
    ) if x_intersect is not None and y_intersect is not None else False

    return is_intersecting, (x_intersect, y_intersect)

def get_slope_intercept_intersection(line1, line2):
    # Convert Point objects to tuples
    line1 = ((line1[0].x, line1[0].y), (line1[1].x, line1[1].y))
    line2 = ((line2[0].x, line2[0].y), (line2[1].x, line2[1].y))

    intersection_result = intersect_slope_intercept(line1, line2)
    if intersection_result[0]:
        return "Lines intersect using Slope-Intercept Method!"
    else:
        return "Lines do not intersect using Slope-Intercept Method."

class ConvexHullVisualization:
    def __init__(self, root):
        self.root = root
        self.root.title("Convex Hull Visualization")

        self.canvas = tk.Canvas(root, width=1200, height=350, bg="lightblue")
        self.canvas.pack()

        self.points = []
        self.convex_hull_points = []

        self.generate_points_button = tk.Button(
            root, text="Generate Random Points", command=self.generate_points
        )
        self.generate_points_button.pack(pady=10)

        self.clear_canvas_button = tk.Button(
            root, text="Clear Canvas", command=self.clear_canvas
        )
        self.clear_canvas_button.pack(pady=10)

        self.elapsed_time_label = tk.Label(root, text="")
        self.elapsed_time_label.pack(pady=10)

        self.step_delay = 500

        self.step_index = 0
        self.visualization_running = False

        self.check_intersection_button = tk.Button(
            root, text="Check Line Intersection (Graham Scan)", command=self.check_intersection
        )
        self.check_intersection_button.pack(pady=10)

        self.check_slope_intercept_button = tk.Button(
            root, text="Check Line Intersection (Slope-Intercept)", command=self.check_slope_intercept_intersection
        )
        self.check_slope_intercept_button.pack(pady=10)

        self.result_label = tk.Label(root, text="")
        self.result_label.pack()

        self.algorithm_buttons_frame = tk.Frame(root)
        self.algorithm_buttons_frame.pack(pady=10)

        self.brute_force_button = tk.Button(
            self.algorithm_buttons_frame,
            text="Brute Force",
            command=lambda: self.start_visualization(brute_force),
        )
        self.brute_force_button.pack(side=tk.LEFT, padx=5)

        self.jarvis_march_button = tk.Button(
            self.algorithm_buttons_frame,
            text="Jarvis March",
            command=lambda: self.start_visualization(jarvis_march),
        )
        self.jarvis_march_button.pack(side=tk.LEFT, padx=5)

        self.graham_scan_button = tk.Button(
            self.algorithm_buttons_frame,
            text="Graham Scan",
            command=lambda: self.start_visualization(graham_scan),
        )
        self.graham_scan_button.pack(side=tk.LEFT, padx=5)

        self.quick_elimination_graham_scan_button = tk.Button(
            self.algorithm_buttons_frame,
            text="Quick Elimination + Graham Scan",
            command=lambda: self.start_visualization(quick_elimination_x_graham_scan),
        )
        self.quick_elimination_graham_scan_button.pack(side=tk.LEFT, padx=5)

        self.monotone_chain_button = tk.Button(
            self.algorithm_buttons_frame,
            text="Monotone Chain",
            command=lambda: self.start_visualization(monotone_chain),
        )
        self.monotone_chain_button.pack(side=tk.LEFT, padx=5)

    def generate_points(self):
        self.clear_canvas()
        self.points = []
        canvas_width = self.canvas.winfo_reqwidth()
        canvas_height = self.canvas.winfo_reqheight()

        num_points = random.randint(10, 15)
        for _ in range(num_points):
            x = random.randint(10, canvas_width - 10)
            y = random.randint(10, canvas_height - 10)
            self.points.append(Point(x, y))
            self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill="red")

    def clear_canvas(self):
        self.canvas.delete("all")
        self.points = []
        self.convex_hull_points = []
        self.elapsed_time_label.config(text="")
        self.result_label.config(text="")

    def clear_previous_visualization(self):
        self.canvas.delete("visualization")

    def start_visualization(self, algorithm):
        if not self.points:
            return

        if not self.visualization_running:
            self.visualization_running = True
            self.clear_previous_visualization()
            start_time = timeit.default_timer()
            self.convex_hull_points = algorithm(self.points)
            end_time = timeit.default_timer()
            elapsed_time = (end_time - start_time) * 1000
            elapsed_time_text = f"Elapsed Time: {elapsed_time:.2f} ms"
            self.elapsed_time_label.config(text=elapsed_time_text)

            self.step_index = 0
            self.root.after(self.step_delay, self.visualize_algorithm, algorithm)

    def visualize_algorithm(self, algorithm):
        if self.visualization_running:
            if self.step_index < len(self.convex_hull_points):
                current_point = self.convex_hull_points[self.step_index]
                color = "blue"
                self.canvas.create_oval(
                    current_point.x - 2,
                    current_point.y - 2,
                    current_point.x + 2,
                    current_point.y + 2,
                    fill=color,
                    tags="visualization",
                )

                if self.step_index > 0:
                    prev_point = self.convex_hull_points[self.step_index - 1]
                    self.canvas.create_line(
                        prev_point.x,
                        prev_point.y,
                        current_point.x,
                        current_point.y,
                        fill="green",
                        width=2,
                        tags="visualization",
                    )

                self.step_index += 1
                self.root.after(self.step_delay, self.visualize_algorithm, algorithm)
            else:
                start_point = self.convex_hull_points[0]
                end_point = self.convex_hull_points[-1]
                self.canvas.create_line(
                    end_point.x,
                    end_point.y,
                    start_point.x,
                    start_point.y,
                    fill="purple",
                    width=2,
                    tags="visualization",
                )
                self.visualization_running = False

    def check_intersection(self):
        if len(self.points) < 4:
            self.result_label.config(text="Not enough vertices to form two lines.")
            return

        line1 = [self.points[-4], self.points[-3]]
        line2 = [self.points[-2], self.points[-1]]

        self.canvas.create_line(
            line1[0].x, line1[0].y, line1[1].x, line1[1].y, fill="black"
        )
        self.canvas.create_line(
            line2[0].x, line2[0].y, line2[1].x, line2[1].y, fill="black"
        )

        if doIntersect(line1[0], line1[1], line2[0], line2[1]):
            self.result_label.config(text="Lines intersect!")
        else:
            self.result_label.config(text="Lines do not intersect.")

    def check_slope_intercept_intersection(self):
        if len(self.points) < 4:
            self.result_label.config(text="Not enough vertices to form two lines.")
            return

        line1 = [self.points[-4], self.points[-3]]
        line2 = [self.points[-2], self.points[-1]]

        self.canvas.create_line(
            line1[0].x, line1[0].y, line1[1].x, line1[1].y, fill="black"
        )
        self.canvas.create_line(
            line2[0].x, line2[0].y, line2[1].x, line2[1].y, fill="black"
        )

        intersection_result = get_slope_intercept_intersection(line1, line2)
        self.result_label.config(text=intersection_result)

if __name__ == "__main__":
    root = tk.Tk()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    root.geometry(f"800x600+{int((screen_width - 800) / 2)}+{int((screen_height - 600) / 2)}")
    app = ConvexHullVisualization(root)
    root.mainloop()
