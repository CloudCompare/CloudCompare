# Upstream Synchronization Policy

1. Preserve the upstream remote and upstream branch history.
2. Record the exact upstream commit SHA used for each Pacai milestone.
3. Synchronize into a dedicated `upstream-sync` branch.
4. Review changed files, submodule pointer changes, CMake options, licenses, and security fixes.
5. Merge reviewed changes into `custom/develop`; never force-push shared branches.
6. Keep Pacai code in clearly separated paths where practical (`pacai/`, `apps/PacaiApp/`, `src/Pacai*`).
7. Avoid modifying upstream algorithm files unless necessary. If a patch is unavoidable, keep it small and document the upstream commit and rationale.
8. Maintain an automated upstream-diff report for each sync.
