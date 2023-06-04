#! /usr/bin/env python3
from __future__ import annotations


class Calculator:
    history: list[str]  # history that the calculator should start with

    def __init__(self, history: list[str]):
        self.history = history

    def add(self, a: int, b: int) -> int:
        self.history.append(f"{a} + {b} = {a + b}")
        return a + b

    def divide(self, a: float, b: float) -> float:
        return a / b

    def get_history(self) -> list[str]:
        return self.history


if __name__ == "__main__":
    calculator = Calculator(["8 / 4 = 2"])
    print(calculator.add(3, 5))
    print(calculator.get_history())
