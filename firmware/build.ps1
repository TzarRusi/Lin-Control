$ErrorActionPreference = 'Stop'

$arduinoCli = 'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
$projectRoot = Split-Path $PSScriptRoot -Parent
$sketch = Join-Path $PSScriptRoot 'lin_control'
$output = Join-Path $projectRoot 'build'
# ARM GCC LTO can fail when its working path contains non-ASCII characters.
# Keep the compiler workspace in the user's ASCII-safe temporary directory.
$build = Join-Path $env:TEMP 'lin-control-build'

if (-not (Test-Path -LiteralPath $arduinoCli)) {
    throw "Arduino CLI not found: $arduinoCli"
}

New-Item -ItemType Directory -Path $build -Force | Out-Null
New-Item -ItemType Directory -Path $output -Force | Out-Null

& $arduinoCli compile `
    --fqbn 'STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C8,usb=CDCgen,xserial=disabled,upload_method=OpenOCDSTLink,opt=oslto' `
    --jobs 4 `
    --build-path $build `
    $sketch

if ($LASTEXITCODE -ne 0) { throw 'Build failed' }

Get-ChildItem -LiteralPath $build -File |
    Where-Object { $_.Name -match '^lin_control\.ino\.(bin|elf|hex|map)$' } |
    Copy-Item -Destination $output -Force

Write-Output "Build completed: $output"
