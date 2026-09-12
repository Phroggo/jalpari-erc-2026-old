"""Detect the overhead shelf-column number markers and read their digits.

The markers are ~0.3 m white plates carrying a dark digit, mounted in a
horizontal row above the shelf. Against the (also white) back wall the plate
outline is nearly invisible, so we localise the *digit* directly — it is by far
the darkest thing in the upper image — then classify the crop with the NumPy
MLP.

Pipeline:
1. threshold dark pixels in the upper part of the image
2. keep digit-sized blobs; group the ones that share a row (the marker strip)
3. crop each with context padding, classify with :class:`DigitMLP`
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .digit_mlp import DigitMLP


@dataclass
class MarkerDetection:
    digit: int
    confidence: float
    u: float            # digit centre x (px)
    v: float            # digit centre y (px)
    bbox: tuple         # (x, y, w, h) of the digit in the full image


class MarkerDetector:
    def __init__(self, weights_path: str, min_conf: float = 0.6):
        self.net = DigitMLP.load(weights_path)
        self.min_conf = min_conf

    # ------------------------------------------------------------------
    def detect(self, bgr: np.ndarray, roi_frac: float = 0.6):
        h, w = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        roi_h = int(h * roi_frac)
        roi = gray[:roi_h]

        # dark digits on a bright plate: adaptive + hard cap
        thr = min(110, int(np.percentile(roi, 15)))
        dark = (roi < max(40, thr)).astype(np.uint8) * 255
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        n, _, stats, cent = cv2.connectedComponentsWithStats(dark, 8)
        cand = []
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if bh < 8 or bh > 0.28 * h:
                continue
            if bw < 3 or bw > 0.14 * w:
                continue
            ar = bw / float(bh)
            if ar < 0.10 or ar > 1.6:
                continue
            if area < 25 or area > 0.9 * bw * bh:
                continue
            if x <= 3 or x + bw >= w - 3:        # clipped at the image edge
                continue
            cx, cy = cent[i]
            cand.append((x, y, bw, bh, cx, cy))

        if not cand:
            return []

        group = self._row_group(cand)

        out = []
        for (x, y, bw, bh, cx, cy) in group:
            mx = int(0.7 * bw) + 2
            my = int(0.45 * bh) + 2
            x0, y0 = max(0, x - mx), max(0, y - my)
            x1, y1 = min(w, x + bw + mx), min(roi_h, y + bh + my)
            crop = gray[y0:y1, x0:x1]
            if crop.size == 0:
                continue
            digit, conf = self.net.predict(crop)
            if conf < self.min_conf:
                continue
            out.append(MarkerDetection(digit=digit, confidence=conf,
                                       u=float(cx), v=float(cy),
                                       bbox=(int(x), int(y), int(bw), int(bh))))
        return out

    # ------------------------------------------------------------------
    @staticmethod
    def _row_group(cand):
        """Keep the largest subset whose centres share a y (the marker strip)."""
        if len(cand) <= 1:
            return sorted(cand, key=lambda c: c[4])
        cys = np.array([c[5] for c in cand])
        hs = np.array([c[3] for c in cand])
        tol = max(14.0, float(np.median(hs)) * 1.1)
        best = []
        for cy in cys:
            grp = [c for c in cand if abs(c[5] - cy) < tol]
            if len(grp) > len(best):
                best = grp
        # drop duplicates that overlap heavily (same digit split into strokes)
        best.sort(key=lambda c: c[4])
        merged = []
        for c in best:
            if merged and c[4] - merged[-1][4] < 0.6 * max(c[2], merged[-1][2]):
                p = merged[-1]
                nx0 = min(p[0], c[0]); ny0 = min(p[1], c[1])
                nx1 = max(p[0] + p[2], c[0] + c[2]); ny1 = max(p[1] + p[3], c[1] + c[3])
                merged[-1] = (nx0, ny0, nx1 - nx0, ny1 - ny0,
                              (nx0 + nx1) / 2.0, (ny0 + ny1) / 2.0)
            else:
                merged.append(c)
        return merged
