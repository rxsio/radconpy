from typing import Generic, TypeVar, List, Callable

Delegate = TypeVar("Delegate")


class Event(Generic[Delegate]):

    def __init__(self):
        self._delegates: List[Delegate] = []

    def add(self, delegate: Delegate) -> None:
        if delegate not in self._delegates:
            self._delegates.append(delegate)

    def remove(self, delegate: Delegate) -> None:
        if delegate in self._delegates:
            self._delegates.remove(delegate)

    def clear(self) -> None:
        self._delegates.clear()

    def broadcast(self, *args, **kwargs) -> None:
        for delegate in self._delegates:
            delegate(*args, **kwargs)
