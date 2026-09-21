# SheetNest 2.0

Native desktop software for nesting and laser-cutting preparation of metal sheet parts.

## Current modules

- C++20 nesting core
- DXF import with LINE / ARC / CIRCLE / LWPOLYLINE / POLYLINE
- DXF contour joining, holes and diagnostics
- True-shape NFP with cached union geometry
- Continuous NFP feasibility region
- Bounded/adaptive feasibility candidate search for large parts
- Sheet margin and inter-part gap
- Multiple instances and rotations
- Per-part quantities for mixed DXF jobs
- Stable instanceId/unitId tracking
- Per-instance placement diagnostics with failure stage
- Built-in benchmark: 1-iteration baseline vs configured optimized search
- Nesting result visualization
- Draggable sheet layouts in the workspace
- DXF layout export
- Bodor 3 kW technology table for speed and assist gas
- Cutting-path planning
- Laser cutting time estimate
- Background calculation in the Qt GUI

## Windows build

Install Visual Studio 2022 with the Desktop C++ workload and Qt 6.8.x MSVC 2022 64-bit.

Qt 6.8 supports Windows x64 with MSVC 2022. For a direct CMake build, point CMAKE_PREFIX_PATH to your Qt installation if CMake cannot find Qt automatically.

### Configure and build

PowerShell:

~~~
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DSHEETNEST_BUILD_GUI=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
cmake --install build --config Release --prefix dist
~~~

The GUI executable is:

~~~text
dist/SheetNest.exe
~~~

The install step uses Qt's deployment script, so the required Qt runtime files are placed next to the executable.

### Package ZIP

~~~
cpack --config build/CPackConfig.cmake -C Release
~~~

This creates a Windows ZIP package containing the deployed SheetNest application.

## GUI workflow

1. Load a DXF.
2. Set sheet width/height, edge margin and gap.
3. Set quantity, optimization iterations and allowed rotations.
4. Select material and thickness.
5. The Bodor 3 kW technology module selects the nearest configured cutting speed and assist gas.
6. Run nesting.
7. Inspect and drag complete sheet layouts in the workspace.
8. Review sheet count, utilization, cut length, pierces and estimated laser time.
9. Export the finished layout to DXF.

## Core-only build

Qt is optional for the core CI build. When Qt is unavailable, CMake still builds and tests sheetnest_core.

~~~
cmake -S . -B build -DSHEETNEST_BUILD_GUI=OFF
cmake --build build --config Release
ctest --test-dir build --output-on-failure
~~~

## Continuous integration

- core-build.yml validates the portable C++ core.
- windows-build.yml configures Qt 6.8.3 on Windows, builds the GUI, runs core tests, deploys the Qt runtime and uploads a ZIP artifact.
