"""Index complete, contiguous demonstration windows without inventing actions."""
import numpy as np


def complete_chunk_starts(episodes, horizon):
    if not isinstance(horizon, (int, np.integer)) or horizon < 1:
        raise ValueError("positive integer horizon required")
    episodes = np.asarray(episodes)
    if episodes.ndim != 1:
        raise ValueError("episode indices must be a vector")
    boundaries = np.r_[0, np.flatnonzero(episodes[1:] != episodes[:-1]) + 1, len(episodes)]
    windows = [np.arange(start, end - horizon + 1, dtype=np.int64)
               for start, end in zip(boundaries[:-1], boundaries[1:])
               if end - start >= horizon]
    if not windows:
        raise ValueError(f"no complete action chunks: each episode needs at least {horizon} frames")
    return np.concatenate(windows)
