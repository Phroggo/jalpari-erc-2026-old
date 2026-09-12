"""Colour components and robust RGB-D projection, shared by mission and tests."""
from dataclasses import dataclass
import cv2
import numpy as np

RANGES = {'red': [((0,90,50),(10,255,255)), ((170,90,50),(179,255,255))],
          'green': [((35,65,40),(88,255,255))],
          'blue': [((95,85,40),(135,255,255))],
          'yellow': [((18,85,60),(36,255,255))]}
ROW_HEIGHTS = np.array([1.577, 1.247, .917, .587])


@dataclass
class Detection:
    box: tuple
    pixel: tuple
    point: np.ndarray


def projected_box(points, intrinsic, shape):
    """Visible bounds of a known-size object anchored by a live observation."""
    points = np.asarray(points)
    if np.any(points[:,2] <= .05) or not np.all(np.isfinite(points)):
        return None
    fx,fy,cx,cy = intrinsic
    pixels = points[:,:2]/points[:,2,None]*[fx,fy]+[cx,cy]
    low = np.maximum(np.floor(pixels.min(axis=0)),[0,0]).astype(int)
    high = np.minimum(np.ceil(pixels.max(axis=0)),[shape[1]-1,shape[0]-1]).astype(int)
    if np.any(high <= low):
        return None
    return (*low,*(high-low))


def project(depth, intrinsic, u, v, mask=None):
    """Use object pixels only: surrounding shelf depth biases a narrow spine."""
    h, w = depth.shape
    u, v = int(round(u)), int(round(v))
    if not (0 <= u < w and 0 <= v < h):
        return None
    x0, x1, y0, y1 = max(0,u-2), min(w,u+3), max(0,v-4), min(h,v+5)
    patch = depth[y0:y1, x0:x1]
    valid = np.isfinite(patch) & (patch > .2) & (patch < 8.)
    if mask is not None:
        valid &= mask[y0:y1, x0:x1] > 0
    values = patch[valid]
    if len(values) < 3 or np.percentile(values,90)-np.percentile(values,10) > .10:
        return None
    z = float(np.median(values))
    return np.array([(u-intrinsic[2])*z/intrinsic[0], (v-intrinsic[3])*z/intrinsic[1], z])


def objects(bgr, depth, intrinsic, colour, bin_mode=False, height_transform=None):
    """Find colour blobs and reject shapes/depths that do not fit a book or bin."""
    if bgr.shape[:2] != depth.shape:
        raise ValueError('RGB and depth must have matching registered dimensions')
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = np.zeros(depth.shape, np.uint8)
    for low, high in RANGES[colour]:
        mask |= cv2.inRange(hsv, np.array(low), np.array(high))
    if bin_mode and height_transform is not None:
        v,u = np.indices(depth.shape)
        fx,fy,cx,cy = intrinsic
        valid = np.isfinite(depth)&(depth>.2)&(depth<8.)
        safe_depth = np.where(valid,depth,0.)
        height = (height_transform[2,0]*(u-cx)/fx + height_transform[2,1]*(v-cy)/fy
                  + height_transform[2,2])*safe_depth+height_transform[2,3]
        mask[~(valid & (height>.72) & (height<1.03))] = 0
    # Do not erode a 2 cm spine: at long range it can be only two pixels wide.
    count, labels, stats, centers = cv2.connectedComponentsWithStats(mask, 8)
    result = []
    for i in range(1, count):
        x,y,w,h,area = map(int,stats[i])
        if (area < (80 if bin_mode else 10) or h < 6 or x <= 1 or x+w >= depth.shape[1]-1
                or y <= 1 or y+h >= depth.shape[0]-1):
            continue
        # The official bin is ~31 cm across the robot's view and ~50 cm deep;
        # looking down into it can make its image taller than it is wide.
        if bin_mode and w < .35*h:
            continue
        if not bin_mode and (h < 1.6*w or h > 20*w):
            continue
        u,v = centers[i]
        component = labels==i
        if bin_mode:
            # A hollow bin's centroid can lie in a hole or behind an occlusion.
            # Pick a real component pixel rather than querying background depth.
            ys,xs = np.nonzero(component)
            nearest = np.argmin((xs-u)**2+(ys-v)**2)
            sample_u,sample_v = xs[nearest],ys[nearest]
        else:
            sample_u,sample_v = u,v
        p = project(depth, intrinsic, sample_u, sample_v, component.astype(np.uint8))
        if p is not None and bin_mode and height_transform is not None:
            ys,xs = np.nonzero(component & np.isfinite(depth))
            z = depth[ys,xs]
            fx,fy,cx,cy = intrinsic
            points = np.c_[(xs-cx)*z/fx,(ys-cy)*z/fy,z]
            world = points @ height_transform[:3,:3].T+height_transform[:3,3]
            lower,upper = np.percentile(world,[5,95],axis=0)
            if max((upper-lower)[:2])<.20:
                continue
            centre = (lower+upper)/2
            if upper[0]-lower[0]<.10:
                # Only the facing wall is visible; account for half bin depth.
                centre[0] -= np.sign(height_transform[0,3]-centre[0])*.25
            p = height_transform[:3,:3].T @ (centre-height_transform[:3,3])
        if p is not None:
            result.append(Detection((x,y,w,h), (u,v), p))
    return result


def row_for_height(height, tolerance=.085):
    """Map measured height to row 1–4, even when other books are out of view."""
    index = int(np.argmin(abs(ROW_HEIGHTS-height)))
    return index+1 if abs(ROW_HEIGHTS[index]-height) <= tolerance else None
