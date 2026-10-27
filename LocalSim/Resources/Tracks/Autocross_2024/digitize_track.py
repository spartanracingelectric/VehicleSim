"""Utility: digitizes a scanned FSAE course map's solid red centerline (25ft x 25ft
grid squares) into a smooth, evenly-spaced (x_ft, y_ft) centerline CSV for a lap time
simulation to consume. digitize() is the reusable entry point - see its call at the
bottom of this file for the default Autocross_2024.png -> track_centerline.csv case,
and digitize_autocross_track.py for the Autocross_2026.png case.

Pipeline: threshold the red line pixels -> skeletonize to a 1px-wide centerline
(Zhang-Suen thinning) -> keep the largest connected component (drops small
stray red specks from cone icons/text) -> order the pixels into a single closed
loop via nearest-neighbor walk -> calibrate px->ft scale from the grid line
spacing -> fit a periodic smoothing spline to remove pixel-raster noise ->
resample at even arc-length spacing.

Rerun this (or the autocross variant) if the source map is replaced with a
cleaner/higher-res image.
"""
from pathlib import Path
SCRIPT_DIR = Path(__file__).resolve().parent

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from PIL import Image
from scipy import ndimage
from scipy.spatial import cKDTree
from scipy.interpolate import splprep, splev

IMAGE_PATH = SCRIPT_DIR / "Autocross_2024.png"
OUTPUT_CSV = SCRIPT_DIR / "track_centerline.csv"
OVERLAY_PNG = SCRIPT_DIR / "track_overlay.png"
GRID_SQUARE_FT = 25.0
STEP_FT = 2.0  # output waypoint spacing
TARGET_LENGTH_M = 1000.0  # known actual lap length - the grid-line calibration is only an
                           # estimate (thin/antialiased gridlines, image compression), so the
                           # whole traced loop is uniformly rescaled to match this ground truth

def load_red_mask(image_path):
    img = Image.open(image_path).convert("RGB")
    arr = np.array(img).astype(int)
    r, g, b = arr[:, :, 0], arr[:, :, 1], arr[:, :, 2]
    return (r > 150) & (g < 120) & (b < 120)

def calibrate_scale_ft_per_px(image_path, red_mask):
    """Grid line spacing (in px) calibrates px->ft, using the known 25ft squares."""
    img = Image.open(image_path).convert("RGB")
    arr = np.array(img).astype(int)
    gray = arr.sum(axis=2) / 3.0
    h, w = gray.shape

    colscore = (gray[5:h - 5, :] < 240).sum(axis=0)
    col_peaks = [x for x in range(1, w - 1) if colscore[x] > 60]
    col_diffs = np.diff(col_peaks)
    col_diffs = col_diffs[col_diffs > 3]

    rowscore = (gray[:, 180:880] < 240).sum(axis=1)
    row_peaks = [y for y in range(1, h - 1) if rowscore[y] > 400]
    row_diffs = np.diff(row_peaks)
    row_diffs = row_diffs[row_diffs > 3]

    px_per_square = np.concatenate([col_diffs, row_diffs]).mean()
    return GRID_SQUARE_FT / px_per_square

def zhang_suen_thin(mask):
    img = mask.copy().astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            P = img
            Pu, Pd = np.roll(P, 1, axis=0), np.roll(P, -1, axis=0)
            Pl, Pr = np.roll(P, 1, axis=1), np.roll(P, -1, axis=1)
            Pul, Pur = np.roll(Pu, 1, axis=1), np.roll(Pu, -1, axis=1)
            Pdl, Pdr = np.roll(Pd, 1, axis=1), np.roll(Pd, -1, axis=1)
            p2, p3, p4, p5, p6, p7, p8, p9 = Pu, Pur, Pr, Pdr, Pd, Pdl, Pl, Pul
            B_ = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9
            neighbors = [p2, p3, p4, p5, p6, p7, p8, p9, p2]
            A_ = np.zeros_like(P, dtype=int)
            for i in range(8):
                A_ += ((neighbors[i] == 0) & (neighbors[i + 1] == 1)).astype(int)
            if step == 0:
                cond3, cond4 = (p2 * p4 * p6) == 0, (p4 * p6 * p8) == 0
            else:
                cond3, cond4 = (p2 * p4 * p8) == 0, (p2 * p6 * p8) == 0
            remove = (P == 1) & (B_ >= 2) & (B_ <= 6) & (A_ == 1) & cond3 & cond4
            if remove.any():
                img[remove] = 0
                changed = True
    return img.astype(bool)

def largest_component(skel):
    labeled, n = ndimage.label(skel, structure=np.ones((3, 3), dtype=int))
    sizes = ndimage.sum(skel, labeled, range(1, n + 1))
    main_label = np.argmax(sizes) + 1
    ys, xs = np.where(labeled == main_label)
    return np.column_stack([xs, ys]).astype(float)

def order_as_path(pts, jump_threshold_px=15.0):
    """Greedy nearest-neighbor walk into a single ordered path. Stops at the first
    big jump - a handful of leftover points beyond that are disconnected raster
    noise, not real track.

    Returns (path, is_closed). A course whose bold line actually forms a closed
    loop (e.g. Endurance, which returns to the start pixel) gets is_closed=True and
    the loop closed back to its start point. A point-to-point course (e.g.
    Autocross, whose start and finish are genuinely not connected by the bold
    line - only by a separate thin/light connector that's outside the red
    threshold) gets is_closed=False and is left open: forcing a closing edge
    across a real gap like that draws a straight-line/spline shortcut with no
    relation to the actual track.
    """
    n = len(pts)
    visited = np.zeros(n, dtype=bool)
    tree = cKDTree(pts)

    start_idx = np.lexsort((pts[:, 1], pts[:, 0]))[0]
    order = [start_idx]
    visited[start_idx] = True
    cur = start_idx
    for _ in range(n - 1):
        k = 8
        found = -1
        while found == -1 and k <= n:
            d, idx = tree.query(pts[cur], k=k)
            for dist, j in zip(np.atleast_1d(d), np.atleast_1d(idx)):
                if not visited[j]:
                    found, fd = j, dist
                    break
            k *= 2
        if found == -1 or (order and fd > jump_threshold_px):
            break
        order.append(found)
        visited[found] = True
        cur = found

    path = pts[order]
    closing_gap = np.hypot(*(path[-1] - path[0]))
    is_closed = closing_gap <= jump_threshold_px
    if is_closed:
        path = np.vstack([path, path[0]])
    return path, is_closed

def smooth_and_resample(path_ft, step_ft, is_closed):
    x, y = path_ft[:, 0], path_ft[:, 1]
    per = 1 if is_closed else 0
    tck, _ = splprep([x, y], per=per, s=len(x) * 1.5)
    raw_length = np.sum(np.hypot(np.diff(x), np.diff(y)))
    n_out = max(int(round(raw_length / step_ft)), 3)
    u_fine = np.linspace(0, 1, n_out, endpoint=not is_closed)
    x_out, y_out = splev(u_fine, tck)
    centerline = np.column_stack([x_out, y_out])
    # The actual output length, not the pre-smoothing raw polyline's - the spline
    # smooths out sharp corners, which shortens the path somewhat
    if is_closed:
        closed = np.vstack([centerline, centerline[0]])
        length = np.sum(np.hypot(np.diff(closed[:, 0]), np.diff(closed[:, 1])))
    else:
        length = np.sum(np.hypot(np.diff(centerline[:, 0]), np.diff(centerline[:, 1])))
    return centerline, length

def enforce_direction(centerline, direction):
    """Reorders a closed loop so it runs "cw" or "ccw" (y up), i.e. in the course's
    real driving direction - the pixel walk in order_as_path picks an arbitrary one."""
    x, y = centerline[:, 0], centerline[:, 1]
    signed_area = 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)
    is_ccw = signed_area > 0
    if (direction == "ccw") != is_ccw:
        centerline = centerline[::-1].copy()
    return centerline

def digitize(image_path, output_csv, overlay_png, target_length_m=None,
             step_ft=STEP_FT, legend_label="Digitized centerline", direction=None):
    """Runs the full pipeline for one course map and writes output_csv + overlay_png.

    direction ("cw"/"ccw", closed loops only) sets the output's point order to the
    course's real driving direction, viewed on the map.

    target_length_m, if given, uniformly rescales the traced loop to match a known
    actual lap length (the grid-line calibration is only an estimate - thin/antialiased
    gridlines, image compression). Left as None, the output keeps the grid-calibrated
    scale as-is.
    """
    red_mask = load_red_mask(image_path)
    ft_per_px = calibrate_scale_ft_per_px(image_path, red_mask)
    print(f"Scale: {ft_per_px:.4f} ft/px")

    skel = zhang_suen_thin(red_mask)
    pts_px = largest_component(skel)
    path_px, is_closed = order_as_path(pts_px)
    print(f"Ordered path: {len(path_px)} px points, {'closed loop' if is_closed else 'open (point-to-point) course'}")

    path_ft = path_px * ft_per_px
    path_ft[:, 1] *= -1.0  # flip image row axis so +y is "up"

    centerline, length_ft = smooth_and_resample(path_ft, step_ft, is_closed)
    if is_closed and direction is not None:
        centerline = enforce_direction(centerline, direction)
        print(f"Point order set to the driving direction: {direction}")
    print(f"Traced length (grid calibration, output centerline): {length_ft:.1f} ft ({length_ft * 0.3048:.1f} m)")

    if target_length_m is not None:
        target_length_ft = target_length_m / 0.3048
        correction = target_length_ft / length_ft
        centerline *= correction
        print(f"Rescaled by {correction:.4f} to match known length: {target_length_ft:.1f} ft ({target_length_m:.1f} m)")
    else:
        correction = 1.0
    print(f"Centerline waypoints: {len(centerline)}")

    np.savetxt(output_csv, centerline, delimiter=",", header="x_ft,y_ft", comments="")
    print(f"Saved {output_csv}")

    save_overlay(image_path, centerline, ft_per_px * correction, overlay_png, legend_label)
    print(f"Saved {overlay_png}")

def save_overlay(image_path, centerline, ft_per_px_corrected, overlay_png, legend_label):
    """Plots the final (rescaled) centerline over the source image, for a visual sanity
    check that the traced path actually follows the map's solid red line. The image's
    display extent is scaled by the same correction factor as the centerline, so the two
    still line up even though the centerline no longer matches the image's native
    grid-calibrated scale (see target_length_m)."""
    img = np.array(Image.open(image_path).convert("RGB"))
    h, w = img.shape[:2]

    fig, ax = plt.subplots(figsize=(20, 4))
    ax.imshow(img, extent=[0, w * ft_per_px_corrected, -h * ft_per_px_corrected, 0])
    ax.plot(centerline[:, 0], centerline[:, 1], color="blue", linewidth=1.2, label=legend_label)
    ax.set_aspect("equal")
    ax.set_xlabel("x [ft]")
    ax.set_ylabel("y [ft]")
    ax.set_title("Digitized Track vs. Source Map")
    ax.legend(loc="upper right")
    plt.tight_layout()
    plt.savefig(overlay_png, dpi=150)
    plt.close(fig)

if __name__ == "__main__":
    # The Autocross run on this course layout is driven clockwise on the map
    digitize(IMAGE_PATH, OUTPUT_CSV, OVERLAY_PNG, TARGET_LENGTH_M,
              legend_label="Digitized centerline (used by Autocross.py)", direction="cw")
