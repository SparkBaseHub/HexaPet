param(
    [string]$Uf2Path
)

# 1. Suche nach dem aktiven COM-Port des Pico / Servo 2040
$port = Get-CimInstance Win32_SerialPort | Where-Object { $_.Description -match "Pico|Board|Serial|Pimoroni" } | Select-Object -ExpandProperty DeviceID -First 1

if ($port) {
    Write-Host "[RESET] Gefunden: $port - Sende 1200-Baud Software-Reset..."
    # 1200 Baud triggert den automatischen Bootloader-Sprung im Pico SDK
    $serial = New-Object System.IO.Ports.SerialPort $port, 1200, None, 8, One
    try {
        $serial.Open()
        Start-Sleep -Milliseconds 100
        $serial.Close()
    } catch {}
    Start-Sleep -Seconds 1
}

# 2. Warte kurz, bis das RPI-RP2 Laufwerk erscheint
$drive = $null
for ($i = 0; $i -lt 15; $i++) {
    $drive = Get-Volume | Where-Object { $_.FileSystemLabel -eq "RPI-RP2" } | Select-Object -ExpandProperty DriveLetter -ErrorAction SilentlyContinue
    if ($drive) { break }
    Start-Sleep -Milliseconds 200
}

# 3. Flashen
if ($drive) {
    Write-Host "[FLASH] RPI-RP2 gefunden auf ${drive}:\ - Übertrage Firmware..."
    Copy-Item -Path $Uf2Path -Destination "${drive}:\"
    Write-Host "[FERTIG] Erfolgreich geflasht! Board startet jetzt automatisch."
} else {
    Write-Host "[FEHLER] Board konnte nicht gefunden werden. Falls es noch nie geflasht wurde, einmalig BOOT+RESET drücken."
}