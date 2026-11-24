# BRMesh ESP32 MQTT bridge

ESP32 firmware (PlatformIO, Arduino framework) that controls BRMesh Bluetooth lights from
Home Assistant over MQTT. The light commands are BLE advertisements in the "fastcon" format
(no BLE connection). Based on davidsclee/BRmesh-esp32-mqtt, which was based on
millskyle/BRMesh_homeassistant and ArcadeMachinist/brMeshMQTT. That repo has no license,
so keep the credit in README.md.

Owner's setup: Windows, VS Code + PlatformIO, repo at `H:\dev\BRmesh-esp32-mqtt`,
GitHub `29w211barnes/brmesh-esp32-mqtt` (branch `main`). Lights are RGBW floodlights
(type code A8A1). The broker is on Home Assistant, and Falcon Player (FPP) / xLights are
also on the network.

## Build / flash

- `pio run`: build
- `pio run -t upload`: flash firmware (and the partition table)
- `pio run -t uploadfs`: upload `data/` to LittleFS (needed after changing config.json)
- `pio device monitor`: serial monitor at 115200

## Layout

- `src/main.cpp`: everything (protocol encoding, pairing, HA entities, config)
- `data/config.json`: WiFi/MQTT settings, **gitignored, has real passwords, never commit or print it**
- `data/config.example.json`: committed template
- `partitions.csv`: the 896K data partition is named `spiffs` (subtype spiffs) so LittleFS mounts it by default

## How it works

1. Boot: mount LittleFS and read `/config.json`. If that fails, the LED blinks fast and the
   ESP32 halts. Holding the BOOT button (GPIO0) at power-on clears the saved key and lights.
2. The mesh key (4 bytes) is created once and saved in NVS (`Preferences`, namespace `brmesh`).
   Paired lights (MAC, type, number) are saved there too, so reboots don't need re-pairing.
3. It scans for lights still on the factory default key `5E367BC4` and pairs each one
   (sends it our key plus a mesh number from 1 up), retrying up to `PAIR_ATTEMPTS`.
   The HA button "Rescan for Lights" (`brmesh_rescan`) runs the same scan while running. It sets
   a flag that `loop()` picks up, and if new lights were added it reconnects MQTT, because ArduinoHA
   only publishes discovery configs when it connects.
4. One HA light entity per light, unique ID `brmesh_<light MAC>`.
5. Commands are 8-byte payloads broadcast for `COMMAND_ADVERTISE_MS` (500 ms):
   - `22 <num> <b> 00 00 00 00 00`: on/off/brightness. b=0x80 means on, 0 means off, 1-127 is brightness
   - `72 <num> <0x80|bright> <B> <R> <G> <ctLo> <ctHi>`: colour / colour temperature. The 0x80 bit
     matches scross01/esphome-fastcon. RGB is gamma-corrected (`COLOR_GAMMA`, 2.2; set 1.0 to
     disable). "Washed out" colours turned out to be HA's colour wheel sending pale values (e.g.
     100,255,100), not a firmware bug; exact RGB values from HA render correctly.
   - Debug: typing `raw <hex bytes>` in the serial monitor sends up to 8 raw command bytes.

## Changes made vs the original (and why)

- The light ID used the first 2 bytes of the BLE address (the vendor prefix), so two lights got
  the same ID and HA merged them. It now uses the full 6-byte MAC from the manufacturer data.
- The light number was read through a dangling pointer (`.c_str()` of a temporary). It now uses the number we requested.
- The key was random on every boot and never saved, so the lights were lost after a reboot. Key and lights are now saved.
- Pairing had one 1-second try. It now retries.
- `getLight` threw an exception, which rebooted the ESP32. It now returns nullptr.
- Memory leaks fixed. HAMqtt entity limit raised from 6 to 32. Byte comparison now casts to `uint8_t`.
- ArduinoHA keeps the `const char*` it's given for IDs and names, so those strings are `strdup`'d.
- Colour bug: `setBrightness()` before MQTT connects doesn't store the value, so colour
  commands went out with brightness 0. Brightness, on state and last colour are now tracked per
  `LightDevice` and colour commands use those (`sendColor`). A repeat "ON" for a light that's
  already on is skipped, and a brightness change while in colour mode resends the colour.

## Status / open items

- Confirmed on hardware: discovery, pairing and on/off work (one light tested so far).
- **Unconfirmed:** whether colours work after the brightness fix. If they still don't, check the
  byte order of the 0x72 command (B, R, G came from the original code) and whether the
  brightness byte needs 0x80 set.
- Not yet tested with two lights at once (the original problem).
- Doesn't share lights with the BrMesh phone app: pairing replaces the key, so whichever side paired last controls the light.
- Possible future work: DDP receiver (UDP 4048) so xLights/FPP can set colours directly. Only
  1-2 colour changes per show are needed. Use latest-value-wins, send only on change, and give
  HA control back when DDP packets stop. Today HA watches the FPP playlist and sets colours itself.