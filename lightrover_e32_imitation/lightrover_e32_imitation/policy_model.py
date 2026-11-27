"""学習済み方策 (小さなMLP) を numpy だけで動かすためのモジュール。

学習は PyTorch (train.py) で行い、重みを .npz に書き出します。
推論側は numpy だけで動くので、ROS 2 を動かすPCに PyTorch は不要です。

ネットワーク:  特徴量(n_sectors) -> [Linear -> ReLU] x 隠れ層 -> Linear -> tanh
出力は [-1, 1] なので、max_linear / max_angular を掛けて速度に戻します。
"""

import numpy as np


class NumpyPolicy:
    def __init__(self, path):
        d = np.load(path, allow_pickle=False)
        n_layers = int(d['n_layers'])
        self.weights = [d[f'W{i}'].astype(np.float32) for i in range(n_layers)]
        self.biases = [d[f'b{i}'].astype(np.float32) for i in range(n_layers)]
        self.n_sectors = int(d['n_sectors'])
        self.max_range = float(d['max_range'])
        self.laser_yaw = float(d['laser_yaw'])
        self.max_linear = float(d['max_linear'])
        self.max_angular = float(d['max_angular'])

    def __call__(self, features):
        """features: (n_sectors,) または (N, n_sectors) -> (2,) / (N, 2) [linear.x, angular.z]"""
        x = np.asarray(features, dtype=np.float32)
        for i, (w, b) in enumerate(zip(self.weights, self.biases)):
            x = x @ w.T + b
            if i < len(self.weights) - 1:
                x = np.maximum(x, 0.0)
        x = np.tanh(x)
        return x * np.array([self.max_linear, self.max_angular], dtype=np.float32)


def save_npz(path, linear_layers, meta):
    """PyTorch の nn.Linear のリストと設定値を .npz に保存する。"""
    arrays = {}
    for i, layer in enumerate(linear_layers):
        arrays[f'W{i}'] = layer.weight.detach().cpu().numpy()
        arrays[f'b{i}'] = layer.bias.detach().cpu().numpy()
    arrays['n_layers'] = np.array(len(linear_layers))
    for k, v in meta.items():
        arrays[k] = np.array(v)
    np.savez(path, **arrays)
