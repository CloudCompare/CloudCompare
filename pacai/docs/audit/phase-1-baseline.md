# Phase 1 Repository Audit — Initial Baseline

## Product constraints

| Dimension | Decision |
|---|---|
| Product | Pacai CloudCompare |
| Platform | Windows x64 |
| UI | WinUI 3 + C++/WinRT |
| Renderer | OpenGL, initially WGL on Windows |
| Algorithm engine | Reuse CCCoreLib |
| Qt policy | No Qt in Pacai build/runtime |

## Evidence collected from the Fork's `master`

The following files were inspected through the GitHub repository:

- `CMakeLists.txt`
- `libs/CMakeLists.txt`
- `qCC/CMakeLists.txt`
- `ccViewer/CMakeLists.txt`
- `plugins/CMakeLists.txt`
- `BUILD.md`
- `.gitmodules`

### Findings

1. Root `CMakeLists.txt` includes `DeployQt`, adds `libs`, `plugins`, `qCC`, and optionally `ccViewer`. It is the upstream application build, not a Pacai no-Qt build entry.
2. `qCC/CMakeLists.txt` uses `qt6_wrap_ui`, `qt6_add_resources`, and links `Qt6::PrintSupport`, `Qt6::Widgets`, `Qt6::OpenGLWidgets`, and `Qt6::OpenGL`. qCC must not be part of the first Pacai no-Qt target graph.
3. `ccViewer/CMakeLists.txt` also generates Qt UI/resources and links `CCAppCommon`; it is not a ready-made Qt-free viewer.
4. `libs/CMakeLists.txt` groups `CCFbo`, `qCC_db`, `qCC_io`, `qCC_glWindow`, `qPCP`, `CCPluginStub`, `CCPluginAPI`, and `CCAppCommon`. Each target requires an independent dependency audit; names alone do not prove Qt independence.
5. `plugins/CMakeLists.txt` loads core/example plugins and dynamically adds private plugin directories. The existing plugin system must be treated as a Qt-era integration surface until each plugin is inspected.
6. `.gitmodules` declares multiple submodules. The actual checked-out submodule commits must be recorded locally; `.gitmodules` alone does not establish their initialized state.
7. CCCoreLib's own `CMakeLists.txt` exposes `CCCORELIB_USE_QT_CONCURRENT` (default ON) and `CCCORELIB_USE_TBB` (default OFF). Its TBB branch sets QtConcurrent OFF. TBB availability and actual compilation must be verified; merely setting the option is not a successful build.

## Provisional migration classification

| Area | Initial disposition | Confidence |
|---|---|---|
| CCCoreLib | Reuse after isolated no-Qt build | High for CMake option; build unverified |
| qCC UI/application | Reimplement in WinUI 3 | High |
| ccViewer UI | Reimplement | High |
| Existing Qt OpenGL widgets | Replace with WGL/OpenGL renderer host | High |
| qCC_db / qCC_io | Audit target-by-target | Low until transitive graph is built |
| Existing Qt plugins | Extract algorithms/adapters; do not load directly | High |
| Format implementations | Review individually for Qt types and entity coupling | Low |
| Upstream CI/package workflows | Preserve selectively, add Pacai workflow | Medium |

## Required next evidence

- Local `git status`, all remotes, all refs, merge-base and complete ahead/behind comparison.
- Initialized submodule status and pinned SHAs.
- CMake File API target graph after configure.
- Qt include/symbol scan across tracked source and submodules.
- CCCoreLib build on Windows with Qt absent from `CMAKE_PREFIX_PATH` and environment.
- `dumpbin /dependents` or equivalent inspection of generated binaries.
- SPDX/license report for all redistributed code and assets.

## Important limitations

This is an initial static baseline, not a complete audit. No local checkout, Windows compiler, submodule tree, CMake configure, or binary was available during this inspection. No successful build is claimed.
