param(
    [string]$Port = 'COM4',
    [ValidateSet('Info','Ping','Poll','Wake','Monitor','PowerOff')]
    [string]$Action = 'Info',
    [ValidatePattern('^[0-9A-Fa-f]{2}$')]
    [string]$Id = '14',
    [ValidateRange(1,3600)]
    [int]$Seconds = 10,
    [switch]$Power
)

$ErrorActionPreference = 'Stop'
$serial = [IO.Ports.SerialPort]::new(
    $Port,
    115200,
    [IO.Ports.Parity]::None,
    8,
    [IO.Ports.StopBits]::One
)
$serial.NewLine = "`n"
$serial.ReadTimeout = 2500
$serial.WriteTimeout = 500
$serial.DtrEnable = $false

function Read-Reply {
    param([string]$Command)
    $serial.WriteLine($Command)
    $reply = $serial.ReadLine().Trim()
    [pscustomobject]@{ Command = $Command; Reply = $reply }
}

try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $serial.DtrEnable = $true
    $banner = $serial.ReadLine().Trim()
    if ($banner -notmatch '^STM32_LIN_10 READY DISABLED$') {
        throw "Unexpected firmware banner: $banner"
    }
    $banner

    if ($Action -in 'Wake','Poll','Monitor') {
        if (-not $Power) {
            throw "Action $Action requires -Power to enable the 12 V bench supply"
        }
        Read-Reply 'POWER 1'
    }

    switch ($Action) {
        'Info' { Read-Reply 'INFO'; Read-Reply 'POWER?' }
        'Ping' { Read-Reply 'PING' }
        'PowerOff' { Read-Reply 'STOP' }
        'Wake' {
            Read-Reply 'ENABLE 1'
            Read-Reply 'WAKE'
            Read-Reply 'ENABLE 0'
        }
        'Poll' {
            Read-Reply 'ENABLE 1'
            Read-Reply ("POLL " + $Id.ToUpperInvariant())
            Read-Reply 'ENABLE 0'
        }
        'Monitor' {
            Read-Reply 'ENABLE 1'
            $deadline = [DateTime]::UtcNow.AddSeconds($Seconds)
            while ([DateTime]::UtcNow -lt $deadline) {
                Read-Reply 'POWER KEEP'
                foreach ($statusId in '14','1E','32') {
                    Read-Reply ("POLL $statusId")
                }
                Start-Sleep -Milliseconds 100
            }
            Read-Reply 'ENABLE 0'
        }
    }
}
finally {
    if ($serial.IsOpen) {
        try { $serial.WriteLine('STOP'); $null = $serial.ReadLine() } catch {}
        $serial.DtrEnable = $false
        $serial.Close()
    }
    $serial.Dispose()
}
