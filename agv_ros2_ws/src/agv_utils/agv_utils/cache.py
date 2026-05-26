# 缓存工具模块，提供LRU、TTL和FIFO三种线程安全的缓存实现
import threading
import time
from collections import OrderedDict
from typing import Any, Optional


# LRUCache: 最近最少使用缓存，访问时将键移到末尾，满时淘汰最久未使用的键
class LRUCache:
    def __init__(self, maxsize: int = 128):
        self._maxsize = maxsize
        self._cache = OrderedDict()
        self._lock = threading.Lock()

    # 获取缓存值，命中时将键移到末尾表示最近使用
    def get(self, key: Any, default: Any = None) -> Any:
        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
                return self._cache[key]
            return default

    # 放入缓存，已存在则移到末尾，超出容量则淘汰最旧的条目
    def put(self, key: Any, value: Any):
        with self._lock:
            if key in self._cache:
                self._cache.move_to_end(key)
            self._cache[key] = value
            if len(self._cache) > self._maxsize:
                self._cache.popitem(last=False)

    # 清空缓存
    def clear(self):
        with self._lock:
            self._cache.clear()


# TTLCache: 带生存时间的缓存，过期条目在访问时自动淘汰
class TTLCache:
    def __init__(self, maxsize: int = 128, ttl: float = 60.0):
        self._maxsize = maxsize
        self._ttl = ttl
        self._cache = OrderedDict()
        # 记录每个键的写入时间戳
        self._timestamps = {}
        self._lock = threading.Lock()

    # 获取缓存值，过期则自动删除并返回默认值
    def get(self, key: Any, default: Any = None) -> Any:
        with self._lock:
            if key not in self._cache:
                return default
            # 检查是否过期
            if time.time() - self._timestamps[key] > self._ttl:
                del self._cache[key]
                del self._timestamps[key]
                return default
            self._cache.move_to_end(key)
            return self._cache[key]

    # 放入缓存，记录时间戳，超出容量则淘汰最旧条目
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

    # 主动清理所有过期条目
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


# FIFOCache: 先进先出缓存，满时淘汰最早写入的条目
class FIFOCache:
    def __init__(self, maxsize: int = 128):
        self._maxsize = maxsize
        self._cache = OrderedDict()
        self._lock = threading.Lock()

    # 获取缓存值，不影响顺序
    def get(self, key: Any, default: Any = None) -> Any:
        with self._lock:
            return self._cache.get(key, default)

    # 放入缓存，新键超出容量时淘汰最早写入的条目
    def put(self, key: Any, value: Any):
        with self._lock:
            if key not in self._cache and len(self._cache) >= self._maxsize:
                self._cache.popitem(last=False)
            self._cache[key] = value

    # 清空缓存
    def clear(self):
        with self._lock:
            self._cache.clear()
