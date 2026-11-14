# Qt-Free Build Policy

This policy applies to Pacai-owned targets and the final Pacai application/runtime. The untouched upstream CloudCompare qCC/ccViewer build remains Qt-based and is not part of the Pacai product build.

## Forbidden in Pacai targets

- `Qt5::*`, `Qt6::*` imported targets
- `find_package(Qt5 ...)`, `find_package(Qt6 ...)`
- Qt headers and types (`QObject`, `QString`, `QWidget`, `QOpenGLWidget`, etc.)
- Qt code generators (`AUTOMOC`, `AUTOUIC`, `AUTORCC`, `qt6_wrap_ui`, `qt6_add_resources`)
- Qt plugin loading and QtConcurrent

## Enforcement layers

1. Source scan: `Invoke-PacaiAudit.ps1`.
2. CMake graph: inspect direct and transitive target link properties after configure.
3. Build environment: no Qt SDK paths or Qt package configs supplied.
4. Binary: inspect executable and DLL imports (`dumpbin /dependents`).
5. Runtime: verify no Qt DLL is shipped or loaded.

A lexical scan is advisory; the CMake target graph and binary imports are the enforcement evidence.
