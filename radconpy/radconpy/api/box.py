from typing import Generic, TypeVar

T = TypeVar("T")


class Box(Generic[T]):

    def __init__(self, value: T):
        self._value = value

    @property
    def value(self) -> T:
        return self._value

    @value.setter
    def value(self, value: T):
        self._value = value

    def get(self) -> T:
        return self._value

    def set(self, value: T) -> None:
        self._value = value
