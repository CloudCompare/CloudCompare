[CmdletBinding()]
param(
    [string]$RepositoryRoot = (Get-Location).Path,
    [string]$OutputDirectory = ".\artifacts\pacai-audit"
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path $RepositoryRoot).Path
$out = [System.IO.Path]::GetFullPath((Join-Path $repo $OutputDirectory))
New-Item -ItemType Directory -Force -Path $out | Out-Null

function Invoke-GitText([string[]]$GitArgs) {
    $result = & git -C $repo @GitArgs 2>&1
    if ($LASTEXITCODE -ne 0) { return "ERROR: $($result -join "`n")" }
    return ($result -join "`n")
}

$head = Invoke-GitText @("rev-parse", "HEAD")
$status = Invoke-GitText @("status", "--short", "--branch")
$remotes = Invoke-GitText @("remote", "-v")
$branches = Invoke-GitText @("branch", "-avv")
$submodules = Invoke-GitText @("submodule", "status", "--recursive")
$mergeBase = ""
try {
    $mergeBase = Invoke-GitText @("merge-base", "HEAD", "upstream/master")
} catch { $mergeBase = "not available" }

$repoInfo = [ordered]@{
    repository_root = $repo
    audited_at_utc = (Get-Date).ToUniversalTime().ToString("o")
    head = $head
    status = $status
    remotes = $remotes
    branches = $branches
    upstream_master_merge_base = $mergeBase
    submodule_status = $submodules
}
$repoInfo | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 (Join-Path $out "repository.json")
$submodules | Set-Content -Encoding utf8 (Join-Path $out "submodules.txt")

$tracked = & git -C $repo ls-files 2>$null
if ($LASTEXITCODE -ne 0) { throw "git ls-files failed; run this script inside a Git checkout." }

$sourceExtensions = @(".h", ".hpp", ".hh", ".hxx", ".c", ".cc", ".cpp", ".cxx", ".ixx", ".ui", ".qrc", ".cmake", ".txt")
$qtPattern = '(?i)(#\s*include\s*[<"]Q[A-Za-z0-9_/]+|Qt[56]::|find_package\s*\(\s*Qt|qt6?_(add|wrap|generate)|Q_OBJECT|Q_GADGET|Q_PLUGIN_METADATA|QObject|QString|QThread|QtConcurrent|QOpenGL|QWidget|QApplication|QCoreApplication|Q_PLUGIN)'
$sourceRows = [System.Collections.Generic.List[object]]::new()
$cmakeRows = [System.Collections.Generic.List[object]]::new()
$targetRows = [System.Collections.Generic.List[object]]::new()

foreach ($relative in $tracked) {
    $path = Join-Path $repo $relative
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { continue }
    $ext = [System.IO.Path]::GetExtension($path).ToLowerInvariant()
    if ($ext -notin $sourceExtensions) { continue }
    try {
        $lines = Get-Content -LiteralPath $path
        for ($i = 0; $i -lt $lines.Count; $i++) {
            if ($lines[$i] -match $qtPattern) {
                $row = [pscustomobject]@{
                    path = $relative
                    line = $i + 1
                    match = $Matches[0]
                    text = $lines[$i].Trim()
                }
                $sourceRows.Add($row)
                if ($relative -match '(^|/)CMakeLists\.txt$|\.cmake$') { $cmakeRows.Add($row) }
                if ($lines[$i] -match '(?i)target_link_libraries|Qt[56]::|find_package\s*\(\s*Qt') { $targetRows.Add($row) }
            }
        }
    } catch {
        Write-Warning "Could not read $relative : $_"
    }
}

$sourceRows | Export-Csv -NoTypeInformation -Encoding utf8 (Join-Path $out "qt-source-hits.csv")
$cmakeRows | Export-Csv -NoTypeInformation -Encoding utf8 (Join-Path $out "qt-cmake-hits.csv")
$targetRows | Export-Csv -NoTypeInformation -Encoding utf8 (Join-Path $out "cmake-target-hits.csv")

$summary = @"
# Pacai Audit Scan Summary

- Repository: ``$repo``
- HEAD: ``$head``
- Scan time (UTC): $((Get-Date).ToUniversalTime().ToString("o"))
- Qt source hits: $($sourceRows.Count)
- Qt/CMake hits: $($cmakeRows.Count)
- Target/link-related hits: $($targetRows.Count)

## Interpretation

These are lexical scan results over tracked files only. They are not a dependency graph and do not prove that a target can or cannot be built without Qt. Submodule contents not checked into the superproject are not scanned unless initialized and tracked in their own repositories.

Next:
1. Inspect each hit and assign Q0–Q5.
2. Configure the candidate target graph with CMake File API.
3. Build in a clean Windows environment without Qt installed or discoverable.
4. Inspect resulting DLL/EXE dependencies.
"@
$summary | Set-Content -Encoding utf8 (Join-Path $out "audit-summary.md")
Write-Host "Audit scan completed: $out"
