# Build, flash, monitor any of the example projects. Doesn't modify files.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts\idf_run.ps1 -Proj examples\command_test
#   powershell -ExecutionPolicy Bypass -File scripts\idf_run.ps1 -Proj examples\command_test -MonitorSecs 40

param(
    [Parameter(Mandatory=$true)][string]$Proj,
    [string]$Port = 'COM12',
    [int]$MonitorSecs = 30
)

$ErrorActionPreference = 'Continue'

# strip MSys env
$mk = @('MSYSTEM','MSYSTEM_PREFIX','MSYS','MINGW_PREFIX','MINGW_CHOST','MSYSTEM_CARCH','MSYSTEM_CHOST','CHERE_INVOKING','MSYS2_PATH_TYPE')
foreach ($k in $mk) { Remove-Item -Path "Env:$k" -ErrorAction SilentlyContinue }
$pp = $env:PATH -split ';'
$filt = @()
foreach ($p in $pp) { if ($p -and ($p -notlike '*msys*') -and ($p -notlike '*mingw*')) { $filt += $p } }
$env:PATH = $filt -join ';'

. 'C:\Users\wangyu\esp\v5.5\esp-idf\fast_export.ps1'
$env:IDF_CCACHE_ENABLE = '1'

$repoRoot = Split-Path -Parent $PSScriptRoot
$proj = Join-Path $repoRoot ($Proj -replace '/', '\')
$proj = [System.IO.Path]::GetFullPath($proj)

# kill stale monitor holding the port
Get-CimInstance Win32_Process -Filter "Name = 'python.exe' OR Name = 'pythonw.exe'" |
    Where-Object { $_.CommandLine -match 'idf_monitor|esp_idf_monitor|COM12' } |
    ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }
Start-Sleep -Milliseconds 500

Write-Host ('=== project: ' + $proj + ' ===') -ForegroundColor Yellow

Write-Host '-- set-target esp32c3 --' -ForegroundColor Cyan
idf.py -C $proj set-target esp32c3 2>&1 | Select-Object -Last 3 | ForEach-Object { Write-Host '  ' + $_ }
if ($LASTEXITCODE -ne 0) { Write-Error ('set-target failed rc=' + $LASTEXITCODE); exit 1 }

Write-Host '-- build --' -ForegroundColor Cyan
idf.py -C $proj build 2>&1 | Select-Object -Last 4 | ForEach-Object { Write-Host '  ' + $_ }
if ($LASTEXITCODE -ne 0) { Write-Error ('build failed rc=' + $LASTEXITCODE); exit 1 }

Write-Host ('-- flash to ' + $Port + ' --') -ForegroundColor Cyan
idf.py -C $proj -p $Port -b 460800 flash 2>&1 |
    Select-Object -Last 6 | ForEach-Object { Write-Host '  ' + $_ }
if ($LASTEXITCODE -ne 0) { Write-Error ('flash failed rc=' + $LASTEXITCODE); exit 1 }

$logDir = Join-Path $repoRoot '_trylogs'
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }
$logTag = (Split-Path -Leaf $Proj)
$logFile = Join-Path $logDir ($logTag + '.log')
$errFile = Join-Path $logDir ($logTag + '.err')
if (Test-Path $logFile) { Remove-Item -Force $logFile }
if (Test-Path $errFile) { Remove-Item -Force $errFile }

Write-Host ('-- monitor ' + $MonitorSecs + 's -> ' + $logFile + ' --') -ForegroundColor Cyan
$mp = Start-Process -FilePath 'python.exe' `
    -ArgumentList @("$env:IDF_PATH\tools\idf.py", '-C', $proj, '-p', $Port, 'monitor') `
    -RedirectStandardOutput $logFile -RedirectStandardError $errFile `
    -PassThru -WindowStyle Hidden
Start-Sleep -Seconds $MonitorSecs
if ($mp -and -not $mp.HasExited) { Stop-Process -Id $mp.Id -Force -ErrorAction SilentlyContinue }

Write-Host ('-- ' + $logTag + ' log --') -ForegroundColor Cyan
if (Test-Path $logFile) {
    Get-Content $logFile | ForEach-Object { Write-Host '  ' + $_ }
}
Write-Host ''
Write-Host ('Log: ' + $logFile) -ForegroundColor Green
