# Flash a single orientation, then monitor for N seconds.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts\idf_flash_one.ps1 -Tx 7 -Rx 6
#   powershell -ExecutionPolicy Bypass -File scripts\idf_flash_one.ps1 -Tx 6 -Rx 7 -Port COM12 -MonitorSecs 12

param(
    [Parameter(Mandatory=$true)][int]$Tx,
    [Parameter(Mandatory=$true)][int]$Rx,
    [string]$Port = 'COM12',
    [int]$MonitorSecs = 12
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
$proj   = Join-Path $repoRoot 'examples\esp32c3_motor_test'
$logDir = Join-Path $repoRoot '_trylogs'
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }

$name = ('TX' + $Tx + '_RX' + $Rx)
Write-Host ''
Write-Host ('=== ' + $name + ' ===') -ForegroundColor Yellow

# make sure no stale monitor/python holds the port
Get-Process -Name python,pythonw -ErrorAction SilentlyContinue |
    Where-Object { $_.MainWindowTitle -match 'monitor' -or $_.Path -match 'idf_monitor' } |
    Stop-Process -Force -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 500

@(
    'CONFIG_IDF_TARGET="esp32c3"',
    'CONFIG_TWAI_ISR_IN_IRAM=y',
    'CONFIG_COMPILER_OPTIMIZATION_SIZE=y',
    ('CONFIG_ZDT_CAN_TX_GPIO=' + $Tx),
    ('CONFIG_ZDT_CAN_RX_GPIO=' + $Rx),
    'CONFIG_ZDT_MOTOR_ADDR=1',
    'CONFIG_ZDT_CAN_BAUDRATE=500000'
) | Set-Content -Encoding ASCII (Join-Path $proj 'sdkconfig.defaults')

Remove-Item -Force (Join-Path $proj 'sdkconfig') -ErrorAction SilentlyContinue

Write-Host '-- reconfigure + build --' -ForegroundColor Cyan
idf.py -C $proj reconfigure 2>&1 | Select-Object -Last 2 | ForEach-Object { Write-Host '  ' + $_ }
idf.py -C $proj build 2>&1 | Select-Object -Last 4 | ForEach-Object { Write-Host '  ' + $_ }
if ($LASTEXITCODE -ne 0) { Write-Host ('BUILD FAILED for ' + $name) -ForegroundColor Red; exit 1 }

Write-Host ('-- flash to ' + $Port + ' --') -ForegroundColor Cyan
idf.py -C $proj -p $Port -b 460800 flash 2>&1 |
    Select-Object -Last 6 | ForEach-Object { Write-Host '  ' + $_ }
if ($LASTEXITCODE -ne 0) { Write-Host ('FLASH FAILED for ' + $name) -ForegroundColor Red; exit 1 }

$logFile = Join-Path $logDir ($name + '.log')
$errFile = Join-Path $logDir ($name + '.err')
if (Test-Path $logFile) { Remove-Item -Force $logFile }
if (Test-Path $errFile) { Remove-Item -Force $errFile }

Write-Host ('-- monitor ' + $MonitorSecs + 's -> ' + $logFile + ' --') -ForegroundColor Cyan
$mp = Start-Process -FilePath 'python.exe' `
    -ArgumentList @("$env:IDF_PATH\tools\idf.py", '-C', $proj, '-p', $Port, 'monitor') `
    -RedirectStandardOutput $logFile -RedirectStandardError $errFile `
    -PassThru -WindowStyle Hidden
Start-Sleep -Seconds $MonitorSecs
if ($mp -and -not $mp.HasExited) { Stop-Process -Id $mp.Id -Force -ErrorAction SilentlyContinue }

Write-Host ('-- ' + $name + ' log (full) --') -ForegroundColor Cyan
if (Test-Path $logFile) {
    Get-Content $logFile | ForEach-Object { Write-Host '  ' + $_ }
}
Write-Host ''
Write-Host ('Log: ' + $logFile) -ForegroundColor Green
