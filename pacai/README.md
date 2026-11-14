# Pacai CloudCompare — Phase 1 Audit Kit

Confirmed target: **Windows / WinUI 3 / OpenGL / CCCoreLib reuse / Qt none**.

This package is an audit and migration starter kit, not a modified CloudCompare source tree.
It does not replace upstream `CMakeLists.txt`, delete Qt code, or claim that a no-Qt build has passed.

## Run

Open PowerShell in the root of your CloudCompare checkout:

```powershell
pwsh -ExecutionPolicy Bypass -File .\pacai\scripts\audit\Invoke-PacaiAudit.ps1 `
  -RepositoryRoot . `
  -OutputDirectory .\artifacts\pacai-audit
```

The script produces:
- `repository.json` — Git baseline and submodule status
- `qt-source-hits.csv` — source-level Qt references
- `qt-cmake-hits.csv` — CMake Qt references
- `cmake-target-hits.csv` — Qt-related target/link references
- `submodules.txt` — declared submodules and status
- `audit-summary.md` — initial evidence-based summary

The source scan is deliberately conservative: a textual hit is a lead, not proof of a transitive dependency. CMake configure/build and binary dependency inspection are required for final classification.

## CCCoreLib no-Qt validation

The upstream CCCoreLib CMake project exposes:
- `CCCORELIB_USE_QT_CONCURRENT` (default ON)
- `CCCORELIB_USE_TBB` (default OFF)

The standalone parent project in `pacai/noqt-core` disables QtConcurrent. It requires TBB when `PACAI_ENABLE_TBB=ON`; set it OFF for a serial/no-TBB validation. Use a fully initialized submodule checkout.

```powershell
cmake -S .\pacai\noqt-core -B .\build\pacai-core `
  -G "Visual Studio 17 2022" -A x64 `
  -DPACAI_CCCORELIB_SOURCE_DIR="$PWD\libs\qCC_db\extern\CCCoreLib" `
  -DPACAI_ENABLE_TBB=ON

cmake --build .\build\pacai-core --config Release
```

This validates CCCoreLib only. It does **not** build the CloudCompare qCC application, WinUI host, OpenGL renderer, or Pacai plugins.

## Safety and source-control policy

1. Keep the upstream `master` history intact.
2. Create a dedicated Pacai branch from the audited base only after confirming the local worktree is clean.
3. Preserve the upstream remote and record the merge-base SHA.
4. Do not bulk-delete `qCC`, `ccViewer`, or `plugins`; extract and port functionality module by module.
5. Do not merge upstream changes directly into the Pacai product branch without review.
