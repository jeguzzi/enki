# Changelog in fork https://github.com/jeguzzi/enki

## Branch pybind11

Switched from python-boost to pybind11. The Python module keeps almost the same interface.

- added pyproject.toml
- wheels can now be build using `python -m build -w`
- `Enki::Vector` is now exposed as a numpy array, which it now requires now.
