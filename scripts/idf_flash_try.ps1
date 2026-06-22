# Try both CAN TX/RX orientations on io6/io7: flash each, capture boot log,
# let user compare which one got motor responses.
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File scripts\idf_flash_try.ps1
#   powershell -ExecutionPolicy Bypass -File scripts\idf_flash_try.ps1 -Port COM12 -MonitorSecs 10

param(
    [string]$Port = 'COM12',
    [int]$MonitorSecs = 10
)

$ErrorActionPreference = 'Continue'

# strip MSys env (idf.py refuses to run under MSys/MinGW)
$mk = @('MSYSTEM','MSYSTEM_PREFIX','MSYS','MINGW_PREFIX','MINGW_CHOST','MSYSTEM_CARCH','MSYSTEM_CHOST','CHERE_INVOKING','MSYS2_PATH_TYPE')
foreach ($k in $mk) { Remove-Item -Path "Env:$k" -ErrorAction SilentlyContinue }
$pp = $env:PATH -split ';'
$filt = @()
foreach ($p in $pp) { if ($p -and ($p -notlike '*msys*') -and ($p -notlike '*mingw*')) { $filt += $p } }
$env:PATH = $filt -join ';'

. 'C:\Users\wangyu\esp\v5.5\esp-idf\fast_export.ps1'
$env:IDF_CCACHE_ENABLE = '1'

$proj   = 'C:\Users\wangyu\Desktop\tmp\zdt_x42s_can\examples\esp32c3_motor_test'
$logDir = 'C:\Users\wangyu\Desktop\tmp\zdt_x42s_can\_trylogs'
if (-not (Test-Path $logDir)) { New-Item -ItemType Directory -Path $logDir | Out-Null }

$tries = @(
    @{ name = 'TX6_RX7'; tx = 6; rx = 7 },
    @{ name = 'TX7_RX6'; tx = 7; rx = 6 }
)

foreach ($t in $tries) {
    Write-Host ''
    Write-Host '========================================' -ForegroundColor Yellow
    Write-Host ('  TRY ' + $t.name + '  (TX=GPIO' + $t.tx + ' RX=GPIO' + $t.rx + ')') -ForegroundColor Yellow
    Write-Host '========================================' -ForegroundColor Yellow

    # rewrite sdkconfig.defaults with this orientation's Kconfig pins.
    # Removing sdkconfig forces reconfigure to re-read defaults.
    @(
        'CONFIG_IDF_TARGET="esp32c3"',
        'CONFIG_TWAI_ISR_IN_IRAM=y',
        'CONFIG_COMPILER_OPTIMIZATION_SIZE=y',
        ('CONFIG_ZDT_CAN_TX_GPIO=' + $t.tx),
        ('CONFIG_ZDT_CAN_RX_GPIO=' + $t.rx),
        'CONFIG_ZDT_MOTOR_ADDR=1',
        'CONFIG_ZDT_CAN_BAUDRATE=500000'
    ) | Set-Content -Encoding ASCII (Join-Path $proj 'sdkconfig.defaults')

    Remove-Item -Force (Join-Path $proj 'sdkconfig') -ErrorAction SilentlyContinue

    Write-Host '-- reconfigure + build --' -ForegroundColor Cyan
    idf.py -C $proj reconfigure 2>&1 | Select-Object -Last 3 | ForEach-Object { Write-Host '  ' + $_ }
    idf.py -C $proj build 2>&1 | Select-Object -Last 4 | ForEach-Object { Write-Host '  ' + $_ }
    if ($LASTEXITCODE -ne 0) { Write-Host ('BUILD FAILED for ' + $t.name) -ForegroundColor Red; continue }

    Write-Host ('-- flash to ' + $Port + ' --') -ForegroundColor Cyan
    idf.py -C $proj -p $Port -b 460800 flash 2>&1 |
        Select-Object -Last 6 | ForEach-Object { Write-Host '  ' + $_ }
    if ($LASTEXITCODE -ne 0) { Write-Host ('FLASH FAILED for ' + $t.name) -ForegroundColor Red; continue }

    # monitor via idf.py in background; kill after MonitorSecs.
    $logFile = Join-Path $logDir ($t.name + '.log')
    $errFile = Join-Path $logDir ($t.name + '.err')
    if (Test-Path $logFile) { Remove-Item -Force $logFile }
    if (Test-Path $errFile) { Remove-Item -Force $errFile }
    Write-Host ('-- monitor ' + $MonitorSecs + 's -> ' + $logFile + ' --') -ForegroundColor Cyan

    $mp = Start-Process -FilePath 'python.exe' `
        -ArgumentList @("$env:IDF_PATH\tools\idf.py", '-C', $proj, '-p', $Port, 'monitor') `
        -RedirectStandardOutput $logFile -RedirectStandardError $errFile `
        -PassThru -WindowStyle Hidden
    Start-Sleep -Seconds $MonitorSecs
    if ($mp -and -not $mp.HasExited) { Stop-Process -Id $mp.Id -Force -ErrorAction SilentlyContinue }

    Write-Host ('-- last 40 lines of ' + $t.name + ' --') -ForegroundColor Cyan
    if (Test-Path $logFile) {
        Get-Content $logFile -Tail 40 | ForEach-Object { Write-Host '  ' + $_ }
    }
}

Write-Host ''
Write-Host 'Logs saved to: ' $logDir -ForegroundColor Green
Write-Host 'Compare TX6_RX7.log vs TX7_RX6.log. The correct orientation shows:' -ForegroundColor Green
Write-Host '  - no Bus-Off recovery spam' -ForegroundColor Green
Write-Host '  - "rx from 0x.." lines (motor replied)' -ForegroundColor Green
Write-Host '  - step 4 read_pos loop returns data, not timeouts' -ForegroundColor Green
