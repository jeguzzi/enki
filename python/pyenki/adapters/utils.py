from __future__ import annotations


def smod(value: int, n: int) -> int:
    return int((value + 2**(n - 1)) % 2**n - 2**(n - 1))


def umod(value: int, n: int) -> int:
    return int(value % 2**n)
