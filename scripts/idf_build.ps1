# scripts/idf_build.ps1
# Build zdt_x42s_can esp32c3 example with ESP-IDF v5.5 + ccache.
#
# 1. dot-source fast_export.ps1 to activate ESP-IDF v5.5
# 2. explicitly enable ccache (IDF_CCACHE_ENABLE=1) so cached builds are fast
# 3. strip MSys env vars (otherwise idf.py refuses to run)
# 4. print ccache stats before/after to confirm cache is used
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts\idf_build.ps1
#   add -Clean to wipe build/managed_components/sdkconfig and rebuild from scratch

param(
    [switch]$Clean
)

# --- 1. strip MSys env ---
$msysKeys = @('MSYSTEM','MSYSTEM_PREFIX','MSYS','MINGW_PREFIX','MINGW_CHOST','MSYSTEM_CARCH','MSYSTEM_CHOST','CHERE_INVOKING','MSYS2_PATH_TYPE')
foreach ($k in $msysKeys) { Remove-Item -Path "Env:$k" -ErrorAction SilentlyContinue }
$pathParts = $env:PATH -split ';'
$filtered = @()
foreach ($p in $pathParts) { if ($p -and ($p -notlike '*msys*') -and ($p -notlike '*mingw*')) { $filtered += $p } }
$env:PATH = $filtered -join ';'

# --- 2. activate ESP-IDF v5.5 ---
. 'C:\Users\wangyu\esp\v5.5\esp-idf\fast_export.ps1'

# --- 3. enable ccache explicitly (fallback if user session lacks it) ---
$env:IDF_CCACHE_ENABLE = '1'
if (-not $env:CCACHE_DIR) { $env:CCACHE_DIR = "$env:USERPROFILE\.ccache" }

$cc = Get-Command ccache -ErrorAction SilentlyContinue
if (-not $cc) {
    Write-Warning 'ccache not found in PATH; builds will not be cached.'
} else {
    Write-Host ('ccache: ' + $cc.Source) -ForegroundColor Cyan
    Write-Host ('IDF_CCACHE_ENABLE=' + $env:IDF_CCACHE_ENABLE + '  CCACHE_DIR=' + $env:CCACHE_DIR) -ForegroundColor Cyan
}

$repoRoot = Split-Path -Parent $PSScriptRoot
$proj = Join-Path $repoRoot 'examples\esp32c3_motor_test'

if ($Clean) {
    Write-Host '--- cleaning build / managed_components / sdkconfig ---' -ForegroundColor Yellow
    foreach ($d in @('build','managed_components')) {
        $p = Join-Path $proj $d
        if (Test-Path $p) { Remove-Item -Recurse -Force $p }
    }
    foreach ($f in @('dependencies.lock','sdkconfig')) {
        $p = Join-Path $proj $f
        if (Test-Path $p) { Remove-Item -Force $p }
    }
}

function Print-CcacheStats($tag) {
    if (-not $cc) { return }
    Write-Host ('--- ccache stats ' + $tag + ' ---') -ForegroundColor DarkGray
    $lines = & ccache -s 2>&1 | Select-Object -First 8
    foreach ($ln in $lines) { Write-Host ('  ' + $ln) -ForegroundColor DarkGray }
}

if ($cc) { & ccache -z *> $null }   # reset session counters
Print-CcacheStats 'BEFORE'

Write-Host '--- set-target esp32c3 ---' -ForegroundColor Yellow
idf.py -C $proj set-target esp32c3 2>&1 | Select-Object -Last 5
if ($LASTEXITCODE -ne 0) { Write-Error ('set-target failed rc=' + $LASTEXITCODE); exit $LASTEXITCODE }

Write-Host '--- build ---' -ForegroundColor Yellow
$sw = [System.Diagnostics.Stopwatch]::StartNew()
idf.py -C $proj build 2>&1 | Select-Object -Last 12
$rc = $LASTEXITCODE
$sw.Stop()
Write-Host ('build rc=' + $rc + '  elapsed=' + [math]::Round($sw.Elapsed.TotalSeconds,1) + 's') -ForegroundColor Cyan

Print-CcacheStats 'AFTER'

exit $rc
