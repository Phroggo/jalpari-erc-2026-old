#!/usr/bin/env python3
"""Train the shelf-marker digit MLP on the synthetic dataset.

    python3 src/erc_solution/training/gen_dataset.py
    python3 src/erc_solution/training/train_digit_mlp.py

Writes ``erc_solution/models/digit_mlp.npz`` (the runtime weights) and prints a
held-out accuracy / confusion matrix for the report.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(os.path.join(HERE, '..')))
from erc_solution.perception.digit_mlp import DigitMLP, N_CLASSES  # noqa: E402


def main():
    ds = os.path.join(HERE, 'dataset.npz')
    if not os.path.exists(ds):
        sys.exit('run gen_dataset.py first')
    d = np.load(ds)
    X, y = d['X'], d['y']
    n = len(y)
    ntr = int(n * 0.9)
    Xtr, ytr, Xte, yte = X[:ntr], y[:ntr], X[ntr:], y[ntr:]

    net = DigitMLP(seed=0)
    net.train(Xtr, ytr, epochs=40, lr=0.05, batch=128)

    pred = net.forward(Xte).argmax(1)
    acc = float((pred == yte).mean())
    cm = np.zeros((N_CLASSES, N_CLASSES), int)
    for t, p in zip(yte, pred):
        cm[t, p] += 1
    print(f'\nheld-out accuracy: {acc * 100:.2f}%  (n={len(yte)})')
    print('confusion matrix (rows=true digit 1..5, cols=pred):')
    print(cm)

    out_dir = os.path.normpath(os.path.join(HERE, '..', 'models'))
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, 'digit_mlp.npz')
    net.save(out)
    print(f'\nsaved {out}')


if __name__ == '__main__':
    main()
