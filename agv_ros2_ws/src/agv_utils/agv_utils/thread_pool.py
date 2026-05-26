# 线程池和异步任务队列工具模块，提供线程池、异步任务队列和速率限制器
import threading
import queue
import time
from concurrent.futures import ThreadPoolExecutor, Future
from typing import Callable, Any, Iterable, Optional


# ThreadPool: 线程池封装，基于concurrent.futures.ThreadPoolExecutor
class ThreadPool:
    def __init__(self, num_workers: int = 4):
        self._executor = ThreadPoolExecutor(max_workers=num_workers)
        self._num_workers = num_workers

    # 提交任务到线程池，返回Future对象
    def submit(self, func: Callable, *args, **kwargs) -> Future:
        return self._executor.submit(func, *args, **kwargs)

    # 批量提交任务，对可迭代对象中的每个元素执行函数，返回结果列表
    def map(self, func: Callable, iterable: Iterable) -> list:
        return list(self._executor.map(func, iterable))

    # 关闭线程池，可选择是否等待所有任务完成
    def shutdown(self, wait: bool = True):
        self._executor.shutdown(wait=wait)


# AsyncTaskQueue: 异步任务队列，支持阻塞式获取和等待所有任务完成
class AsyncTaskQueue:
    def __init__(self, maxsize: int = 100):
        self._queue = queue.Queue(maxsize=maxsize)
        self._pending_tasks = 0
        self._lock = threading.Lock()
        # 条件变量，用于等待所有任务完成
        self._all_tasks_done = threading.Condition(self._lock)

    # 放入任务，队列满时返回False
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

    # 获取任务，当所有任务处理完毕时通知等待线程
    def get(self, timeout: Optional[float] = None) -> Any:
        item = self._queue.get(timeout=timeout)
        with self._lock:
            self._pending_tasks -= 1
            if self._pending_tasks == 0:
                self._all_tasks_done.notify_all()
        return item

    # 阻塞等待所有任务完成
    def join(self):
        with self._all_tasks_done:
            while self._pending_tasks > 0:
                self._all_tasks_done.wait()


# RateLimiter: 速率限制器，控制函数调用的最大频率
class RateLimiter:
    def __init__(self, max_rate: float):
        self._min_interval = 1.0 / max_rate if max_rate > 0 else 0.0
        self._lock = threading.Lock()
        self._last_execution = 0.0

    # 等待直到满足速率限制条件，确保调用间隔不低于最小间隔
    def wait(self):
        with self._lock:
            now = time.monotonic()
            elapsed = now - self._last_execution
            if elapsed < self._min_interval:
                time.sleep(self._min_interval - elapsed)
            self._last_execution = time.monotonic()
