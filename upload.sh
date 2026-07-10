#!/usr/bin/env bash
# Upload firmware + LittleFS ke ESP32 (DevKit V1, partisi Huge APP)
set -e
cd "$(dirname "$0")"

PORT="${1:-/dev/ttyUSB0}"
FQBN="esp32:esp32:esp32:PartitionScheme=huge_app"
SKETCH="SistemKontrolKelembapanBudidayaJamur"

ARDUINO_CLI="$(command -v arduino-cli || echo "$HOME/bin/arduino-cli")"
MKLFS="$(ls "$HOME"/.arduino15/packages/esp32/tools/mklittlefs/*/mklittlefs | head -1)"
ESPTOOL="$(ls "$HOME"/.arduino15/packages/esp32/tools/esptool_py/*/esptool | head -1)"

echo "== 1/4 Compile =="
"$ARDUINO_CLI" compile --fqbn "$FQBN" --output-dir build "$SKETCH"

echo "== 2/4 Upload firmware ($PORT) =="
"$ARDUINO_CLI" upload --fqbn "$FQBN" --input-dir build -p "$PORT" "$SKETCH"

echo "== 3/4 Buat image LittleFS =="
"$MKLFS" -c "$SKETCH/data" -p 256 -b 4096 -s 917504 build/littlefs.bin

echo "== 4/4 Flash LittleFS @0x310000 =="
"$ESPTOOL" --chip esp32 --port "$PORT" --baud 460800 write_flash 0x310000 build/littlefs.bin

echo "Selesai! Buka serial monitor 115200 utk melihat IP dashboard."
