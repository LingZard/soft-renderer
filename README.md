# C++17 XMake Project

This project is a standard C++17 application scaffolded with xmake. It includes:
- C++17 support
- xmake build system
- Source files in `src/`
- (Planned) Unit tests in `tests/`

## Build

```sh
xmake
```

## Run

```sh
xmake run
```

## Configure C++ Standard (already set to C++17)

```sh
xmake f --cxxflags='-std=c++17'
```

## Add Tests

Place your test files in the `tests/` directory. Catch2 is recommended for unit testing.

---

For more information, see the [xmake documentation](https://xmake.io/).
