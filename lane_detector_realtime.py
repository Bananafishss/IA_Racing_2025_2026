# lane_detector_realtime.py
# 6.23 mm / px  => 0.00623 m/px


import cv2
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler


# # ===== Camera calibration (hardcoded) - DONKEY CAR 160x120 =====
# CAMERA_MATRIX = np.array([
#     [7.769135602155642e+01, 0.0, 8.459660983546296e+01],
#     [0.0, 7.707079327160494e+01, 5.731111111111111e+01],
#     [0.0, 0.0, 1.0]
# ], dtype=np.float32)

# DISTORTION_COEFFICIENTS = np.array([
#     -3.085783342913964722e-01,
#     9.440662542263485169e-02,
#     5.233856342166535829e-03,
#     3.498670331708648128e-03,
#     -1.246511308177137949e-02
# ], dtype=np.float32)

# ===== Merge threshold (in pixels) when a single thick lane is split into two clusters =====
MERGE_CLOSE_CLUSTERS_PX = 15.0

# ===== Minimum valid lane width threshold (in pixels) to avoid collapsing into a center line =====
MIN_VALID_LANE_WIDTH_PX = 12.0

# # ===== Global scale (meters per pixel) =====
METERS_PER_PIXEL = 0.00623

# ===== Single-lane prediction shift distance (in meters) =====
PREDICT_SHIFT_M = 0.6158  # 61.58 cm

# ===== ROI =====
ROI_TOP_RATIO = 0.5
ROI_BOTTOM_RATIO = 1.0
ROI_LEFT_SHIFT = 0.04
ROI_RIGHT_SHIFT = 0.04

# ===== DBSCAN clustering parameters =====
DBSCAN_EPS = 0.25
DBSCAN_MIN_SAMPLES = 3
MIN_CLUSTER_POINTS = 50

# ===== Number of output points per lane =====
N_POINTS = 30


def shift_pixels_from_meters(meters: float) -> float:
    """Convert distance from meters to pixels."""
    return float(meters / METERS_PER_PIXEL)


# =============================================== #
#                   BEV
# =============================================== #
def bev_perspective(img):
    """Apply Bird’s Eye View (perspective transform)."""
    h, w = img.shape[:2]
    src_points = np.float32([
        [w * 0.05, h * 0.7],
        [w * 0.95, h * 0.7],
        [w * 0.70, h * 0.45],
        [w * 0.30, h * 0.45]
    ])
    dst_points = np.float32([
        [w * 0.25, h],
        [w * 0.75, h],
        [w * 0.75, 0],
        [w * 0.25, 0]
    ])
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    return cv2.warpPerspective(img, M, (w, h))


# =============================================== #
#                    ROI Mask
# =============================================== #
def roi_mask(shape):
    """Generate ROI polygon mask."""
    h, w = shape[:2]
    bot_y = min(int(h * ROI_BOTTOM_RATIO), h - 1)
    top_y = max(0, min(int(h * ROI_TOP_RATIO), h - 1))

    bottom_left = (0, bot_y)
    bottom_right = (w - 1, bot_y)
    top_left = (max(0, int(w * ROI_LEFT_SHIFT)), top_y)
    top_right = (min(w - 1, int(w * (1 - ROI_RIGHT_SHIFT))), top_y)
    top_center = (w // 2, max(0, int(top_y - (h * 0.15))))

    poly = np.array([[bottom_left, top_left, top_center, top_right, bottom_right]], dtype=np.int32)
    mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(mask, poly, 255)
    return mask


# =============================================== #
#    Keep only lane regions
# =============================================== #
def filter_lane_regions(binary, bandwidth_ratio=0.15, min_peak_dist_ratio=0.25):
    """Filter binary image to keep lane regions using histogram peaks."""
    h, w = binary.shape
    counts = np.sum(binary > 0, axis=0).astype(np.float32)
    counts = cv2.GaussianBlur(counts.reshape(1, -1), (1, 9), 0).ravel()

    left_peak = int(np.argmax(counts[: w // 2]))
    right_peak = int(np.argmax(counts[w // 2:]) + w // 2)

    if abs(right_peak - left_peak) < int(w * min_peak_dist_ratio):
        mid = w // 2
        left_peak, right_peak = int(mid * 0.7), int(mid * 1.3)

    off = int(w * bandwidth_ratio)
    mask = np.zeros_like(binary)

    for y in range(h):
        scale = 1.0 + 1.5 * (1 - y / h)
        lw = int(off * scale)
        rw = int(off * 1.3 * scale)
        mask[y, max(0, left_peak - lw): min(w, left_peak + lw)] = 255
        mask[y, max(0, right_peak - rw): min(w, right_peak + rw)] = 255

    out = cv2.bitwise_and(binary, mask)
    out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1)
    return out


# =============================================== #
#      Soft cleaning
# =============================================== #
def clean_lane_lines_soft(binary, min_area=60, min_aspect_ratio=2.0,
                          left_keep_ratio=0.45, right_keep_ratio=0.55,
                          morph_kernel_size=2):
    """Remove noise while preserving lane-like components."""
    h, w = binary.shape
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    cleaned = np.zeros_like(binary)

    for i in range(1, nb_components):
        _, _, w_box, h_box, area = stats[i]
        cx, _ = centroids[i]
        aspect_ratio = h_box / (w_box + 1e-6)

        is_left = cx < w * left_keep_ratio
        is_right = cx > w * right_keep_ratio

        if is_left or is_right or aspect_ratio > min_aspect_ratio or area >= min_area:
            cleaned[output == i] = 255

    k = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, k, iterations=1)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, k, iterations=1)
    return cleaned


def merge_lane_segments(binary, left_ratio=0.5):
    """Merge fragmented lane segments on left/right halves."""
    h, w = binary.shape
    mid_x = int(w * left_ratio)

    left_side = binary[:, :mid_x]
    right_side = binary[:, mid_x:]

    k = np.ones((5, 5), np.uint8)
    left_filled = cv2.morphologyEx(left_side, cv2.MORPH_CLOSE, k, iterations=2)
    right_filled = cv2.morphologyEx(right_side, cv2.MORPH_CLOSE, k, iterations=2)

    result = np.zeros_like(binary)
    result[:, :mid_x] = left_filled
    result[:, mid_x:] = right_filled
    return result


# =============================================== #
#              Skeleton
# =============================================== #
def skeletonization(binary, low=30, high=90):
    """Extract skeleton from binary lane mask."""
    edges = cv2.Canny(binary, low, high)
    k = np.ones((3, 3), np.uint8)
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, k, iterations=1)

    # éœ€è¦� opencv-contrib-python
    skel = cv2.ximgproc.thinning(closed)

    skel[:2, :], skel[-2:, :], skel[:, :2], skel[:, -2:] = 0, 0, 0, 0
    return skel


# =============================================== #
#                    Clustering
# =============================================== #
def clustering(edges):
    """Cluster skeleton points into lane candidates."""
    ys, xs = np.where(edges > 0)
    if len(xs) == 0:
        return []

    points = np.column_stack((xs, ys))
    points_scaled = StandardScaler().fit_transform(points)

    db = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(points_scaled)
    labels = db.labels_

    clusters_list = []
    for cid in np.unique(labels):
        if cid == -1:
            continue
        c = points[labels == cid]
        if len(c) >= MIN_CLUSTER_POINTS:
            clusters_list.append(c)

    return clusters_list


def merge_close_clusters_by_x(clusters, merge_px=45.0):
    """
    Merge clusters with similar x-centroids.
    Used to fix the case where one lane is split into two parallel edges.
    """
    if len(clusters) <= 1:
        return clusters

    centroids = np.array([float(np.mean(c[:, 0])) for c in clusters], dtype=np.float32)
    order = np.argsort(centroids)
    clusters_sorted = [clusters[i] for i in order]

    merged = [clusters_sorted[0]]
    for c in clusters_sorted[1:]:
        if abs(float(np.mean(c[:, 0])) - float(np.mean(merged[-1][:, 0]))) < merge_px:
            merged[-1] = np.vstack([merged[-1], c])
        else:
            merged.append(c)
    return merged


# ===== Utility: polynomial helpers =====
def poly_x_at_y(poly, y):
    """Evaluate polynomial x at given y."""
    a, b, c = poly
    return a * y * y + b * y + c


def mid_poly(p1, p2):
    """Compute mid polynomial between two lanes."""
    return ((p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0, (p1[2] + p2[2]) / 2.0)


def poly_points(poly, y_min, y_max, n_points):
    """Generate sampled points along polynomial."""
    y_vals = np.linspace(y_min, y_max, n_points)
    x_vals = poly[0] * y_vals**2 + poly[1] * y_vals + poly[2]
    return [{"x": int(x), "y": int(y)} for x, y in zip(x_vals, y_vals)]


def shifted_poly(poly, dx):
    """
    x = a y^2 + b y + c
    å¹³ç§» x -> x + dx  ç­‰ä»·äºŽ c -> c + dx
    """
    a, b, c = poly
    return (a, b, c + dx)


def _angle_deg_from_polyfit(poly, y_ref: float) -> float:
    """poly = [a,b,c] ; slope = dx/dy = 2*a*y + b ; angle = atan(slope)"""
    a, b, _c = poly
    slope = 2.0 * float(a) * float(y_ref) + float(b)
    return float(np.degrees(np.arctan(slope)))


class LaneDetector:
    """
    Usage (DonkeyCar part):
        detector = LaneDetector()
        out = detector.process(frame_bgr)
    """
    def __init__(self):
        #Store last valid steering value when two real lanes are detected.        self.last_two_lane_steer_norm = 0.0
        self.last_two_lane_valid = False

    def process(self, frame_bgr, last_steer: float = 0.0):
        if frame_bgr is None:
            return self._empty_output()

        # 1) undistort
        #img = cv2.undistort(frame_bgr, CAMERA_MATRIX, DISTORTION_COEFFICIENTS, None, CAMERA_MATRIX)

        # 2) BEV
        bev = bev_perspective(frame_bgr)

        # 3) HSV threshold
        hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv, np.array([0, 0, 180]), np.array([180, 60, 255]))
        mask_yellow = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([35, 255, 255]))
        binary = cv2.bitwise_and(mask_white, cv2.bitwise_not(mask_yellow))

        # 4) ROI in BEV space
        mask_BEV = bev_perspective(roi_mask(frame_bgr.shape))
        binary = cv2.bitwise_and(binary, mask_BEV)

        # 5) filter + clean + merge
        binary = filter_lane_regions(binary, bandwidth_ratio=0.15, min_peak_dist_ratio=0.25)
        binary = clean_lane_lines_soft(binary)
        binary = merge_lane_segments(binary, left_ratio=0.5)

        # 6) skeleton + clustering
        edges = skeletonization(binary, low=30, high=90)
        clusters_list = clustering(edges)
        clusters_list = merge_close_clusters_by_x(clusters_list, merge_px=MERGE_CLOSE_CLUSTERS_PX)

        # 7) JSON computation
        h, w = bev.shape[:2]
        y_bottom = float(h)
        x_car = w / 2.0

        shift_px = shift_pixels_from_meters(PREDICT_SHIFT_M)

        clusters_effective = list(clusters_list)
        predicted_lane_added = False
        single_lane_side_by_history = None

        if len(clusters_list) == 1:
            c0 = clusters_list[0]
            ys0, xs0 = c0[:, 1], c0[:, 0]
            _a0, _b0, _c00 = np.polyfit(ys0, xs0, 2)
            mean_x0 = float(np.mean(xs0))

            side_by_position = "left" if mean_x0 < x_car else "right"

            #  steer
            if self.last_two_lane_valid:
                side = "right" if self.last_two_lane_steer_norm > 0.0 else "left"
                single_lane_side_by_history = side
            else:
                side = side_by_position
                single_lane_side_by_history = None  

        
            dx = (-shift_px) if side == "right" else (+shift_px)

            pred_cluster = c0.copy()
            pred_cluster[:, 0] = pred_cluster[:, 0] + dx

            clusters_effective = [c0, pred_cluster]
            predicted_lane_added = True

        n_eff = len(clusters_effective)

        lane_data = []
        left_distance = None
        right_distance = None
        offset = None
        center_curvatures = None
        y_samples = None
        lane_width_px = None

        for i, cluster in enumerate(clusters_effective):
            if cluster is None or len(cluster) < 4:
                continue

            ys, xs = cluster[:, 1], cluster[:, 0]
            a, b, c = np.polyfit(ys, xs, 2)

            y_vals = np.linspace(np.min(ys), np.max(ys), N_POINTS)
            x_vals = a * y_vals**2 + b * y_vals + c

            y_near = float(np.max(y_vals))
            slope = float(2 * a * y_near + b)
            angle_deg = float(np.degrees(np.arctan(slope)))

            lane_data.append({
                "lane_id": int(i),
                "is_predicted": bool(predicted_lane_added and i == 1),
                "point_count": int(len(y_vals)),
                "polyfit": [float(a), float(b), float(c)],
                "angle_deg": float(angle_deg),
                "slope": float(slope),
                "points": [{"x": int(x), "y": int(y)} for x, y in zip(x_vals, y_vals)]
            })

        # distances / width / offset calculation
        if n_eff >= 1:
            centroids = [float(np.mean(c[:, 0])) for c in clusters_effective]
            order = np.argsort(centroids)

            if n_eff == 1:
                cluster = clusters_effective[0]
                ys, xs = cluster[:, 1], cluster[:, 0]
                a, b, c = np.polyfit(ys, xs, 2)

                x_lane = float(a * y_bottom**2 + b * y_bottom + c)
                mean_x = float(np.mean(xs))

                if mean_x < x_car:
                    left_distance = float(x_car - x_lane)
                else:
                    right_distance = float(x_lane - x_car)

            else:
                infos = []
                for cl in clusters_effective:
                    ys, xs = cl[:, 1], cl[:, 0]
                    a, b, c = np.polyfit(ys, xs, 2)
                    x_bottom_cl = float(a * y_bottom**2 + b * y_bottom + c)
                    mean_x = float(np.mean(xs))
                    infos.append({
                        "cluster": cl,
                        "a": a, "b": b, "c": c,
                        "x_bottom": x_bottom_cl,
                        "mean_x": mean_x
                    })

                infos_sorted = sorted(infos, key=lambda d: float(d["x_bottom"]))
                left_cluster = infos_sorted[0]["cluster"]
                right_cluster = infos_sorted[-1]["cluster"]
                ys_l, xs_l = left_cluster[:, 1], left_cluster[:, 0]
                ys_r, xs_r = right_cluster[:, 1], right_cluster[:, 0]

                a_l, b_l, c_l = np.polyfit(ys_l, xs_l, 2)
                a_r, b_r, c_r = np.polyfit(ys_r, xs_r, 2)

                x_left = float(a_l * y_bottom**2 + b_l * y_bottom + c_l)
                x_right = float(a_r * y_bottom**2 + b_r * y_bottom + c_r)

                left_distance = float(x_car - x_left)
                right_distance = float(x_right - x_car)

                lane_width_px = float(x_right - x_left)

                
                # same-side or too-close: treat as SINGLE real lane (no midline)
                same_side = (x_right < x_car) or (x_left > x_car)

                if (lane_width_px < MIN_VALID_LANE_WIDTH_PX) or same_side:
                    # infos_sorted 
                    if x_left > x_car:
                        # both on right -> pick the one closer to center (smaller x_bottom)
                        chosen = infos_sorted[0]
                    else:
                        # both on left -> pick the one closer to center (larger x_bottom)
                        chosen = infos_sorted[-1]

                    cl = chosen["cluster"]
                    a, b, c = chosen["a"], chosen["b"], chosen["c"]

                    x_lane = float(a * y_bottom**2 + b * y_bottom + c)
                    if x_lane < x_car:
                        left_distance = float(x_car - x_lane)
                        right_distance = None
                    else:
                        right_distance = float(x_lane - x_car)
                        left_distance = None

                    offset = float(x_car - x_lane)
                    lane_width_px = None
                    center_curvatures = None
                    y_samples = None

                    # lane_data 
                    slope_lane = float(2.0 * a * y_bottom + b)
                    angle_lane_deg = float(np.degrees(np.arctan(slope_lane)))
                    y_min = int(np.min(cl[:, 1]))
                    y_max = int(np.max(cl[:, 1]))
                    lane_data = [{
                        "lane_id": 0,
                        "side": ("left" if x_lane < x_car else "right"),
                        "merged_from": "same_side_clusters_pick_one",
                        "point_count": int(N_POINTS),
                        "polyfit": [float(a), float(b), float(c)],
                        "angle_deg": angle_lane_deg,
                        "slope": slope_lane,
                        "points": poly_points((a, b, c), y_min, y_max, N_POINTS)
                    }]

                else:
                    # offset + curvature
                    lane_center = (x_left + x_right) / 2.0
                    offset = float(x_car - lane_center)

                    y_min = int(min(np.min(ys_l), np.min(ys_r)))
                    y_max = int(max(np.max(ys_l), np.max(ys_r)))

                    N_CURV = 10
                    y_samples = np.linspace(y_min, y_max, N_CURV)

                    a_c = (a_l + a_r) / 2.0
                    b_c = (b_l + b_r) / 2.0
                    c_c = (c_l + c_r) / 2.0

                    center_curvatures = []
                    for y in y_samples:
                        x1 = 2 * a_c * y + b_c
                        x2 = 2 * a_c
                        kappa = abs(x2) / (1 + x1 * x1) ** 1.5
                        center_curvatures.append(float(kappa))

        if isinstance(y_samples, np.ndarray):
            y_samples = y_samples.tolist()
        # ===== Update last steering when REAL two lanes are detected (not predicted) =====
        # 
        if (not predicted_lane_added) and (len(clusters_list) >= 2):
            # Only update when we have a valid lane width measurement, to avoid being polluted by the single-lane cases where the predicted lane might be very off
            if lane_width_px is not None:
                self.last_two_lane_steer_norm = float(last_steer)
                self.last_two_lane_valid = True
        # ===== Calculate lane center angle =====
        lane_center_angle_deg = None
        if len(lane_data) == 0:
            lane_center_angle_deg = None
        elif len(lane_data) == 1:
            if "angle_deg" in lane_data[0]:
                lane_center_angle_deg = float(lane_data[0]["angle_deg"])
            elif "polyfit" in lane_data[0]:
                lane_center_angle_deg = _angle_deg_from_polyfit(lane_data[0]["polyfit"], y_bottom)
            else:
                lane_center_angle_deg = None
        else:
            angles = []
            for lane in lane_data:
                if "angle_deg" in lane and lane["angle_deg"] is not None:
                    angles.append(float(lane["angle_deg"]))
                elif "polyfit" in lane and lane["polyfit"] is not None:
                    angles.append(_angle_deg_from_polyfit(lane["polyfit"], y_bottom))
            lane_center_angle_deg = float(sum(angles) / len(angles)) if len(angles) > 0 else None
            if lane_center_angle_deg is not None:
                lane_center_angle_deg = -lane_center_angle_deg

        # metric conversion
        left_distance_m = left_distance * METERS_PER_PIXEL if left_distance is not None else None
        right_distance_m = right_distance * METERS_PER_PIXEL if right_distance is not None else None
        lane_width_m = lane_width_px * METERS_PER_PIXEL if lane_width_px is not None else None
        offset_m = (-offset) * METERS_PER_PIXEL if offset is not None else None

        return {
            "lanes": lane_data,

            # pixel space
            "left_distance_px": left_distance,
            "right_distance_px": right_distance,
            "lane_width_px": lane_width_px,
            "offset_px": offset,

            # metric space
            "left_distance_m": left_distance_m,
            "right_distance_m": right_distance_m,
            "lane_width_m": lane_width_m,
            "offset_m": offset_m,

            "meters_per_pixel": METERS_PER_PIXEL,

            "center_curvatures": center_curvatures,
            "y_samples": y_samples,

            "lane_center_angle_deg": lane_center_angle_deg,

            # debug
            "predicted_lane_added": bool(predicted_lane_added),
            "predict_shift_m": float(PREDICT_SHIFT_M),
            "predict_shift_px": float(shift_px),
            "merge_close_clusters_px": float(MERGE_CLOSE_CLUSTERS_PX),
            "min_valid_lane_width_px": float(MIN_VALID_LANE_WIDTH_PX),
            "last_two_lane_valid": bool(self.last_two_lane_valid),
            "last_two_lane_steer_norm": float(self.last_two_lane_steer_norm),
            "single_lane_side_by_history": single_lane_side_by_history,
        }

    @staticmethod
    def _empty_output():
        return {
            "lanes": [],
            "left_distance_px": None,
            "right_distance_px": None,
            "lane_width_px": None,
            "offset_px": None,
            "left_distance_m": None,
            "right_distance_m": None,
            "lane_width_m": None,
            "offset_m": None,
            "meters_per_pixel": METERS_PER_PIXEL,
            "center_curvatures": None,
            "y_samples": None,
            "lane_center_angle_deg": None,

            # debug
            "predicted_lane_added": False,
            "predict_shift_m": float(PREDICT_SHIFT_M),
            "predict_shift_px": float(shift_pixels_from_meters(PREDICT_SHIFT_M)),
            "merge_close_clusters_px": float(MERGE_CLOSE_CLUSTERS_PX),
            "min_valid_lane_width_px": float(MIN_VALID_LANE_WIDTH_PX),
        }
