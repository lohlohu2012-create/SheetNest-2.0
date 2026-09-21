# SheetNest 2.0

Native Windows desktop metal-sheet nesting and laser cutting software.

## Current architecture

- C++20 core for geometry, DXF, NFP, true-shape nesting, diagnostics and cutting calculations.
- Qt 6 desktop GUI for the industrial CAD/CAM workspace.
- Editable laser technology database for a 3 kW reference setup.
- Local/offline operation; no Node.js or browser runtime is required for the desktop application.

## Build

Core-only build:

```bash
cmake -S . -B build -DSHEETNEST_BUILD_GUI=OFF
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Qt desktop build:

```bash
cmake -S . -B build -DSHEETNEST_BUILD_GUI=ON
cmake --build build --parallel
```

Qt 6 with Widgets and Concurrent is required for `SheetNest2App`.

## Desktop workflow

`DXF → geometry/holes → true-shape/NFP nesting → multi-start parallel search → sheet reduction → diagnostics → DXF export → cutting path/time estimate`

The GUI exposes sheet size, technological edge, gap, material, thickness, assist gas, laser power, cutting speed, piercing time, search restarts and parallelism.

## Laser technology database

The initial `resources/laser_bodor_3kw.csv` contains editable reference values for a 3 kW setup. They are starting values, not guaranteed production parameters. Actual cutting speed depends on material grade, thickness, assist gas, pressure, nozzle, focus, cutting head, optics and machine condition.

The desktop application copies the CSV next to the executable when possible and provides an editor for speed and piercing time.
