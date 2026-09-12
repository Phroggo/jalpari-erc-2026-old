#!/usr/bin/env python3
"""Generate a synthetic training set for the shelf-marker digit classifier.

Source images are the marker textures shipped with the simulation
(``erc_description/models/number_marker/textures/{1..5}.png``). We apply the
kind of distortions the head camera actually produces when viewing an overhead
marker plate at an angle and distance: perspective warp, rotation, scale,
blur, photometric jitter, sensor noise and cluttered backgrounds.

Output: ``dataset.npz`` with ``X`` (N, 1024) float32 and ``y`` (N,) int64
(labels 0..4 == digits 1..5).

Run inside the ERC container:
    python3 src/erc_solution/training/gen_dataset.py
"""
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.normpath(os.path.join(
    HERE, '..', '..', 'erc_description', 'models', 'number_marker', 'textures'))
PATCH = 32
PER_CLASS = 4000


def _warp(img, rng):
    h, w = img.shape[:2]
    m = 0.28
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst = (src + (rng.uniform(-m, m, (4, 2)) * [w, h]).astype(np.float32)).astype(np.float32)
    M = cv2.getPerspectiveTransform(src, dst)
    out = cv2.warpPerspective(img, M, (w, h), borderMode=cv2.BORDER_REPLICATE)
    ang = rng.uniform(-18, 18)
    R = cv2.getRotationMatrix2D((w / 2, h / 2), ang, rng.uniform(0.8, 1.15))
    return cv2.warpAffine(out, R, (w, h), borderMode=cv2.BORDER_REPLICATE)


def _sample(base, rng):
    g = _warp(base, rng)
    # place on a background patch with a border (plate edge / shelf clutter)
    pad = rng.integers(4, 22)
    bg = int(rng.uniform(60, 255))
    canvas = np.full((g.shape[0] + 2 * pad, g.shape[1] + 2 * pad), bg, np.uint8)
    if rng.random() < 0.5:
        canvas += rng.integers(-20, 20, canvas.shape, dtype=np.int16).clip(0, 255).astype(np.uint8)
    canvas[pad:pad + g.shape[0], pad:pad + g.shape[1]] = g
    crop = canvas
    # photometric
    crop = crop.astype(np.float32)
    crop = crop * rng.uniform(0.5, 1.4) + rng.uniform(-40, 40)
    if rng.random() < 0.7:
        k = int(rng.choice([1, 3, 3, 5]))
        crop = cv2.GaussianBlur(crop, (k, k), 0)
    crop = crop + rng.normal(0, rng.uniform(0, 14), crop.shape)
    crop = np.clip(crop, 0, 255).astype(np.uint8)
    if rng.random() < 0.5:  # sometimes invert (dark-on-light vs light-on-dark)
        crop = 255 - crop
    g32 = cv2.resize(crop, (PATCH, PATCH), interpolation=cv2.INTER_AREA).astype(np.float32)
    g32 = (g32 - g32.mean()) / (g32.std() + 1e-6)
    return g32.reshape(-1)


def main():
    if not os.path.isdir(TEX):
        sys.exit(f'marker textures not found at {TEX}')
    rng = np.random.default_rng(1)
    X, y = [], []
    for label, digit in enumerate('12345'):
        p = os.path.join(TEX, f'{digit}.png')
        base = cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2GRAY)
        for _ in range(PER_CLASS):
            X.append(_sample(base, rng))
            y.append(label)
        print(f'digit {digit}: {PER_CLASS} samples')
    X = np.asarray(X, np.float32)
    y = np.asarray(y, np.int64)
    perm = rng.permutation(len(y))
    X, y = X[perm], y[perm]
    out = os.path.join(HERE, 'dataset.npz')
    np.savez(out, X=X, y=y)
    print(f'wrote {out}  X={X.shape}  y={y.shape}')


if __name__ == '__main__':
    main()
