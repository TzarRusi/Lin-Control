$ErrorActionPreference = 'Stop'

$arduinoCli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$projectRoot = Split-Path $PSScriptRoot -Parent
$sketch = Join-Path $PSScriptRoot 'lin_control'
$build = Join-Path $projectRoot 'build'

if (-not (Test-Path -LiteralPath $arduinoCli)) {
    throw "Arduino CLI not found: $arduinoCli"
}

New-Item -ItemType Directory -Path $build -Force | Out-Null

& $arduinoCli compile `
    --fqbn 'STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8,usb=CDCgen,xserial=disabled,upload_method=OpenOCDSTLink,opt=oslto' `
    --jobs 4 `
    --build-path $build `
    $sketch

if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
Write-Output "Build completed: $build"

