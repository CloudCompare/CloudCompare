# CCCoreLib No-Qt Validation — Execution Record

## Status

**Not yet built.** This record distinguishes source/CMake review from an actual compiler result.

- Repository: `qwe6630204/CloudCompare`
- Audit branch: `pacai/audit-v0.1.0-a`
- CCCoreLib submodule path: `libs/qCC_db/extern/CCCoreLib` (declared in the superproject `.gitmodules`)
- Upstream source inspected: `CloudCompare/CCCoreLib` `master:CMakeLists.txt`
- Upstream CMake options observed:
  - `CCCORELIB_USE_QT_CONCURRENT`: default `ON`
  - `CCCORELIB_USE_TBB`: default `OFF`
  - `CCCORELIB_SHARED`: default `ON`
- Upstream's TBB lookup is not REQUIRED. If TBB is absent, the upstream project can continue without linking TBB. The Pacai wrapper therefore performs a REQUIRED TBB lookup before adding CCCoreLib when `PACAI_ENABLE_TBB=ON`.
- The Pacai wrapper now forces QtConcurrent OFF, disables Qt6 package discovery, and checks the CCCoreLib target's direct/interface link properties for Qt imported targets.

## Why a build result is unavailable

The connected GitHub repository API can read tracked text files and create commits, but it does not provide a local initialized Git checkout or a Windows MSVC build host. The current execution environment therefore cannot truthfully report CMake configure, compilation, DLL inspection, or runtime success. The CCCoreLib submodule working tree and its nested dependencies must be present on the Windows build machine.

## Windows x64 validation

Run from the CloudCompare checkout root in a Visual Studio 2022 Developer PowerShell with CMake and Git installed.

### 1. Initialize the exact pinned submodule

```powershell
git submodule update --init --recursive libs/qCC_db/extern/CCCoreLib
if (-not (Test-Path .\libs\qCC_db\extern\CCCoreLib\CMakeLists.txt)) {
    throw "CCCoreLib submodule is not initialized"
}
```

### 2. Serial no-Qt isolation build (first gate)

This validates that CCCoreLib compiles without Qt. It does not claim TBB parallel support.

```powershell
cmake -S .\pacai\noqt-core -B .\build\pacai-cccorelib-noqt `
  -G "Visual Studio 17 2022" -A x64 `
  -DPACAI_CCCORELIB_SOURCE_DIR="$PWD\libs\qCC_db\extern\CCCoreLib" `
  -DPACAI_ENABLE_TBB=OFF

cmake --build .\build\pacai-cccorelib-noqt --config Release --parallel
if ($LASTEXITCODE -ne 0) { throw "CCCoreLib no-Qt build failed" }
```

### 3. Optional oneTBB build (second gate)

Install oneTBB 2022+ and make its CMake package config discoverable (for example, configure `CMAKE_PREFIX_PATH` to the TBB installation prefix).

```powershell
cmake -S .\pacai\noqt-core -B .\build\pacai-cccorelib-noqt-tbb `
  -G "Visual Studio 17 2022" -A x64 `
  -DPACAI_CCCORELIB_SOURCE_DIR="$PWD\libs\qCC_db\extern\CCCoreLib" `
  -DPACAI_ENABLE_TBB=ON `
  -DCMAKE_PREFIX_PATH="$env:TBB_ROOT"
cmake --build .\build\pacai-cccorelib-noqt-tbb --config Release --parallel
if ($LASTEXITCODE -ne 0) { throw "CCCoreLib no-Qt + TBB build failed" }
```

### 4. Binary dependency inspection

Locate the produced `CCCoreLib.dll` under the selected build tree and run:

```powershell
dumpbin /dependents <path-to-CCCoreLib.dll>
```

Acceptance criteria:
- CMake configure succeeds with `CMAKE_DISABLE_FIND_PACKAGE_Qt6=TRUE`.
- `CCCORELIB_USE_QT_CONCURRENT=OFF`.
- For the TBB build, `CCCORELIB_USE_TBB=ON` and `TBB::tbb` is linked.
- CCCoreLib builds successfully.
- No Qt DLL appears in the CCCoreLib direct dependency list.
- Preserve configure/build logs and the exact submodule SHA as CI artifacts.

## Scope boundary

Passing this test proves only that the selected CCCoreLib revision can be built in isolation without Qt. It does not prove that CloudCompare's `qCC_db`, `qCC_io`, qCC application, ccViewer, plugins, WinUI 3 host, or Pacai renderer are Qt-free.
