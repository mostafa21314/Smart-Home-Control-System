# Guest-Comfort-Management-System

An embedded systems project that monitors room occupancy, temperature, and humidity, publishing all data over MQTT via WiFi. Built with ESP-IDF on an ESP32.

---

## System Overview

The system uses two IR break beams and a PIR sensor at a door to detect entries and exits with directional awareness, and a DHT22 sensor to monitor ambient conditions. All data is published to an MQTT broker accessible from any device on the same network.

### Detection Logic

```
ENTRANCE: outer beam breaks → inner beam breaks → PIR detects motion → count++
EXIT:      inner beam breaks → outer beam breaks → count--
```

- Both beams must break within **3 seconds** of each other, otherwise the event is discarded
- A **2-second cooldown** applies after every detected entrance or exit
- Room is OCCUPIED when `count > 0`, EMPTY when `count == 0`

---

## Hardware Components

| Component | Purpose |
|---|---|
| ESP32 | Main microcontroller |
| IR phototransistor × 2 | Break beam receivers (outer and inner) |
| IR LED × 2 | Break beam transmitters (battery-powered, driven at 38 kHz) |
| PIR sensor (HC-SR501) | Confirms entrance direction |
| DHT22 / AM2302 | Temperature and humidity sensing |
| Mosquitto | MQTT broker running on a laptop |

---

## Pin Connections

### IR Break Beams

The transmitters are powered independently from a battery and driven at **38 kHz** (use a second MCU or 555 timer). The receivers connect to the ESP32 as follows:

**Outer receiver (GPIO 13) — outside the door:**
```
3.3V ──[10kΩ]──┬── GPIO 13
               Collector
           (phototransistor)
               Emitter
               │
              GND
```

**Inner receiver (GPIO 14) — inside the door:**
```
3.3V ──[10kΩ]──┬── GPIO 14
               Collector
           (phototransistor)
               Emitter
               │
              GND
```

### PIR Sensor (GPIO 25)

```
5V    ── PIR VCC
GND   ── PIR GND
GPIO 25 ── PIR OUT
```

### DHT22 / AM2302 (GPIO 26)

```
3.3V  ── DHT22 pin 1 (VCC)
GPIO 26 ── DHT22 pin 2 (DATA)
NC    ── DHT22 pin 3
GND   ── DHT22 pin 4 (GND)
```

Add a **10 kΩ pull-up** between DATA and 3.3V if using the bare 4-pin DHT22. The pre-wired AM2302 (3-wire version) has it built in.

---

## MQTT Topics

| Topic | Values | Description |
|---|---|---|
| `smarthome/room001/room` | `OCCUPIED` / `EMPTY` | Room occupancy state |
| `smarthome/room001/count` | Integer | Number of people currently in the room |
| `smarthome/room001/temperature` | Float (°C) | Published every 5 seconds |
| `smarthome/room001/humidity` | Float (%) | Published every 5 seconds |
| `smarthome/room001/command` | `LIGHTS_ON` / `LIGHTS_OFF` / `STATUS` | Inbound commands |

---

## Configuration

Before building, update these defines in [main/main.c](main/main.c):

```c
#define WIFI_SSID   "your-hotspot-name"
#define WIFI_PASS   "your-password"
#define MQTT_HOST   "broker-ip-address"   // your laptop's IP on the same network
#define MQTT_USER   "admin"
#define MQTT_PASS   "your-mqtt-password"
```

Find your laptop's IP with:
```powershell
ipconfig | findstr /i "IPv4"
```

---

## Environment Setup

Activate the ESP-IDF environment before running any commands (once per terminal session):

```powershell
. "C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1"
```

Or launch **ESP-IDF v6.0.1 PowerShell** from the Start Menu.

> `.vscode/settings.json` and `.vscode/c_cpp_properties.json` are not tracked by git — each developer configures them locally for their own machine.

---

## MQTT Broker Setup (Mosquitto)

Mosquitto must be running on the laptop before flashing. In an **Administrator** PowerShell:

```powershell
net start mosquitto
```

Verify it's listening:
```powershell
netstat -an | findstr "1883"
```

You should see `0.0.0.0:1883 LISTENING`.

---

## Building and Flashing

Find your ESP32's COM port:
```powershell
Get-PnpDevice -Class Ports | Where-Object Status -eq 'OK' | Select-Object FriendlyName
```

Flash and open the serial monitor:
```powershell
idf.py -p COM5 flash monitor
```

Replace `COM5` with your actual port. Press **Ctrl+]** to exit the monitor.

**Step by step if preferred:**
```powershell
idf.py build
idf.py -p COM5 flash
idf.py -p COM5 monitor
```

---

## Monitoring via MQTT Explorer

1. Connect your phone or laptop to the **same WiFi network** as the ESP32
2. Open [MQTT Explorer](https://mqtt-explorer.com) and connect:
   - Host: your laptop's IP
   - Port: `1883`
   - Username: `admin`
   - Password: *(as configured)*
3. Subscribe to `smarthome/room001/#` to see all topics live
