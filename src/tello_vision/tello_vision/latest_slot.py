"""Thread-safe latest-value slot that intentionally drops stale work."""

from __future__ import annotations

import threading
from typing import Generic, TypeVar

T = TypeVar('T')


class LatestSlot(Generic[T]):
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._value: T | None = None
        self._sequence = 0
        self._closed = False

    def put(self, value: T) -> int:
        with self._condition:
            self._value = value
            self._sequence += 1
            self._condition.notify()
            return self._sequence

    def wait_next(self, previous_sequence: int, timeout: float = 0.5) -> tuple[int, T | None]:
        with self._condition:
            self._condition.wait_for(
                lambda: self._closed or self._sequence > previous_sequence,
                timeout=timeout,
            )
            if self._closed:
                return self._sequence, None
            return self._sequence, self._value

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()
