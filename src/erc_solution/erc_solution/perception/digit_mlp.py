"""Tiny NumPy MLP for shelf-marker digit classification (classes 1..5).

The network uses NumPy, with OpenCV for crop resizing. It reads a 32x32
grayscale patch through two hidden layers:

    1024 -> 128 -> 64 -> 5   (ReLU, softmax output, cross-entropy loss)

Training data is generated synthetically from the marker textures shipped in
``erc_description`` (see ``training/gen_dataset.py`` and
``training/train_digit_mlp.py``). Weights are stored as an ``.npz`` and loaded
at runtime by :class:`DigitMLP.load`.
"""

from __future__ import annotations

import numpy as np

PATCH = 32          # network input is PATCH x PATCH grayscale
N_CLASSES = 5       # digits 1..5  -> indices 0..4


def preprocess_patch(gray: np.ndarray) -> np.ndarray:
    """Normalise a HxW uint8/float grayscale crop to a flat (1024,) float32 vector.

    Steps: resize to 32x32, per-image mean/std standardisation. Robust to the
    marker appearing dark-on-light or light-on-dark.
    """
    import cv2

    g = gray.astype(np.float32)
    if g.shape != (PATCH, PATCH):
        g = cv2.resize(g, (PATCH, PATCH), interpolation=cv2.INTER_AREA)
    m = float(g.mean())
    s = float(g.std()) + 1e-6
    g = (g - m) / s
    return g.reshape(-1)


def _relu(x):
    return np.maximum(x, 0.0)


def _softmax(x):
    x = x - x.max(axis=1, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=1, keepdims=True)


class DigitMLP:
    """2-hidden-layer MLP. Holds parameters and does forward / (optional) train."""

    def __init__(self, hidden=(128, 64), seed=0):
        rng = np.random.default_rng(seed)
        d0 = PATCH * PATCH
        h1, h2 = hidden
        # He initialisation
        self.W1 = rng.standard_normal((d0, h1)).astype(np.float32) * np.sqrt(2.0 / d0)
        self.b1 = np.zeros(h1, np.float32)
        self.W2 = rng.standard_normal((h1, h2)).astype(np.float32) * np.sqrt(2.0 / h1)
        self.b2 = np.zeros(h2, np.float32)
        self.W3 = rng.standard_normal((h2, N_CLASSES)).astype(np.float32) * np.sqrt(2.0 / h2)
        self.b3 = np.zeros(N_CLASSES, np.float32)

    # ---- inference -------------------------------------------------------
    def forward(self, X: np.ndarray) -> np.ndarray:
        """X: (N, 1024) -> probabilities (N, 5)."""
        self._z1 = X @ self.W1 + self.b1
        self._a1 = _relu(self._z1)
        self._z2 = self._a1 @ self.W2 + self.b2
        self._a2 = _relu(self._z2)
        self._z3 = self._a2 @ self.W3 + self.b3
        return _softmax(self._z3)

    def predict(self, gray_patch: np.ndarray):
        """Classify one grayscale crop. Returns (digit 1..5, confidence)."""
        x = preprocess_patch(gray_patch)[None, :]
        p = self.forward(x)[0]
        k = int(p.argmax())
        return k + 1, float(p[k])

    # ---- training (used offline only) ----------------------------------
    def train(self, X, y, epochs=40, lr=0.05, batch=128, wd=1e-4, seed=0, log=print):
        rng = np.random.default_rng(seed)
        n = X.shape[0]
        Y = np.eye(N_CLASSES, dtype=np.float32)[y]
        vel = {k: np.zeros_like(getattr(self, k)) for k in
               ('W1', 'b1', 'W2', 'b2', 'W3', 'b3')}
        mom = 0.9
        for ep in range(epochs):
            idx = rng.permutation(n)
            tot = 0.0
            for s in range(0, n, batch):
                b = idx[s:s + batch]
                xb, yb = X[b], Y[b]
                p = self.forward(xb)
                m = xb.shape[0]
                loss = -np.sum(yb * np.log(p + 1e-9)) / m
                tot += loss * m
                # backprop
                gz3 = (p - yb) / m
                gW3 = self._a2.T @ gz3 + wd * self.W3
                gb3 = gz3.sum(0)
                ga2 = gz3 @ self.W3.T
                gz2 = ga2 * (self._z2 > 0)
                gW2 = self._a1.T @ gz2 + wd * self.W2
                gb2 = gz2.sum(0)
                ga1 = gz2 @ self.W2.T
                gz1 = ga1 * (self._z1 > 0)
                gW1 = xb.T @ gz1 + wd * self.W1
                gb1 = gz1.sum(0)
                for k, g in (('W1', gW1), ('b1', gb1), ('W2', gW2),
                             ('b2', gb2), ('W3', gW3), ('b3', gb3)):
                    vel[k] = mom * vel[k] - lr * g
                    setattr(self, k, getattr(self, k) + vel[k])
            log(f'  epoch {ep + 1:3d}/{epochs}  loss={tot / n:.4f}')

    # ---- persistence --------------------------------------------------
    def save(self, path):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2,
                 W3=self.W3, b3=self.b3)

    @classmethod
    def load(cls, path):
        d = np.load(path)
        m = cls()
        for k in ('W1', 'b1', 'W2', 'b2', 'W3', 'b3'):
            setattr(m, k, d[k].astype(np.float32))
        return m
