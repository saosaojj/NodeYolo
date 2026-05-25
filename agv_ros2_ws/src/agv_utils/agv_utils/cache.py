import threading
import time
from collections import OrderedDict
from typing import Any, Optional


class LRUCache:
    def __init__(self, maxsize: int = 128):
        self._maxsize = maxsize
        self._cache = OrderedDict()
        self._lock = threading.Lock()

    def get(self, key: Any, default: Any = None) -> Any:
        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
                return self._cache[key]
            return default

    def put(self, key: Any, value: Any):
        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
            self._cache[key] = value
            if len(self._cache) > self._maxsize:
                self._cache.popitem(last=False)

    def clear(self):
        with self._lock:
            self._cache.clear()


class TTLCache:
    def __init__(self, maxsize: int = 128, ttl: float = 60.0):
        self._maxsize = maxsize
        self._ttl = ttl
        self._cache = OrderedDict()
        self._timestamps = {}
        self._lock = threading.Lock()

    def get(self, key: Any, default: Any = None) -> Any:
        with self._lock:
            if key not in self._cache:
                return default
            if time.time() - self._timestamps[key] > self._ttl:
                del self._cache[key]
                del self._timestamps[key]
                return default
            self._cache.move_to_end(key)
            return self._cache[key]

    def put(self, key: Any, value: Any):
        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
            self._cache[key] = value
            self._timestamps[key] = time.time()
            if len(self._cache) > self._maxsize:
                oldest = next(iter(self._cache))
                del self._cache[oldest]
                del self._timestamps[oldest]

    def cleanup(self):
        with self._lock:
            current_time = time.time()
            expired_keys = [
                k for k, ts in self._timestamps.items()
                if current_time - ts > self._ttl
            ]
            for key in expired_keys:
                del self._cache[key]
                del self._timestamps[key]


class FIFOCache:
    def __init__(self, maxsize: int = 128):
        self._maxsize = maxsize
        self._cache = OrderedDict()
        self._lock = threading.Lock()

    def get(self, key: Any, default: Any = None) -> Any:
        with self._lock:
            return self._cache.get(key, default)

    def put(self, key: Any, value: Any):
        with self._lock:
            if key not in self._cache and len(self._cache) >= self._maxsize:
                self._cache.popitem(last=False)
            self._cache[key] = value

    def clear(self):
        with self._lock:
            self._cache.clear()
