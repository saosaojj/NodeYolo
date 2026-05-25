import threading
import queue
import time
from concurrent.futures import ThreadPoolExecutor, Future
from typing import Callable, Any, Iterable, Optional


class ThreadPool:
    def __init__(self, num_workers: int = 4):
        self._executor = ThreadPoolExecutor(max_workers=num_workers)
        self._num_workers = num_workers

    def submit(self, func: Callable, *args, **kwargs) -> Future:
        return self._executor.submit(func, *args, **kwargs)

    def map(self, func: Callable, iterable: Iterable) -> list:
        return list(self._executor.map(func, iterable))

    def shutdown(self, wait: bool = True):
        self._executor.shutdown(wait=wait)


class AsyncTaskQueue:
    def __init__(self, maxsize: int = 100):
        self._queue = queue.Queue(maxsize=maxsize)
        self._pending_tasks = 0
        self._lock = threading.Lock()
        self._all_tasks_done = threading.Condition(self._lock)

    def put(self, item: Any, timeout: Optional[float] = None) -> bool:
        with self._lock:
            self._pending_tasks += 1
        try:
            self._queue.put(item, timeout=timeout)
            return True
        except queue.Full:
            with self._lock:
                self._pending_tasks -= 1
            return False

    def get(self, timeout: Optional[float] = None) -> Any:
        item = self._queue.get(timeout=timeout)
        with self._lock:
            self._pending_tasks -= 1
            if self._pending_tasks == 0:
                self._all_tasks_done.notify_all()
        return item

    def join(self):
        with self._all_tasks_done:
            while self._pending_tasks > 0:
                self._all_tasks_done.wait()


class RateLimiter:
    def __init__(self, max_rate: float):
        self._min_interval = 1.0 / max_rate if max_rate > 0 else 0.0
        self._lock = threading.Lock()
        self._last_execution = 0.0

    def wait(self):
        with self._lock:
            now = time.monotonic()
            elapsed = now - self._last_execution
            if elapsed < self._min_interval:
                time.sleep(self._min_interval - elapsed)
            self._last_execution = time.monotonic()
