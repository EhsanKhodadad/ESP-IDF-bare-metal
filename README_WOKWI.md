# Single Door Control - Wokwi Simulator

This version is optimized for the Wokwi ESP32 simulator with visual feedback using LEDs and push buttons.

## Hardware Configuration

### Inputs (Push Buttons - Active Low)
- **GPIO 13**: OPEN button (Green)
- **GPIO 12**: CLOSE button (Blue)  
- **GPIO 14**: STOP button (Red)

### Outputs (LEDs)
- **GPIO 26**: Red LED - Motor Clockwise (Opening)
- **GPIO 27**: Yellow LED - Motor Counter-clockwise (Closing)
- **GPIO 25**: Green LED - Motor Stopped
- **GPIO 2**: Blue LED - Status blink on state changes

## How to Run in Wokwi

### Option 1: Wokwi VS Code Extension

1. Install the [Wokwi for VS Code](https://marketplace.visualstudio.com/items?itemName=wokwi.wokwi-vscode) extension
2. Update `main/CMakeLists.txt` to use `single_door_wokwi.c`:
   ```cmake
   idf_component_register(SRCS "single_door_wokwi.c"
                          PRIV_REQUIRES spi_flash esp_timer driver
                          INCLUDE_DIRS "")
   ```
3. Build the project:
   ```powershell
   idf.py build
   ```
4. Press **F1** → **Wokwi: Start Simulator**

### Option 2: Wokwi Web (wokwi.com)

1. Create a new ESP32 project at [wokwi.com](https://wokwi.com)
2. Copy the contents of `single_door_wokwi.c` to the editor
3. Copy the `diagram.json` to configure the hardware
4. Click "Start Simulation"

## Operation

1. **System starts** → Green LED on (Motor STOP), Door CLOSED
2. **Press OPEN** → Red LED on, door opens for 5 seconds
3. **Press CLOSE** → Yellow LED on, door closes for 5 seconds
4. **Press STOP** → Green LED on, motor stops immediately
5. **Blue LED** blinks briefly on every state transition

## Serial Monitor Output

The system logs all state changes to the serial monitor:
```
[0 ms] Door State: CLOSED
[1234 ms] Motor: FORWARD
[1234 ms] Door State: OPENING
[6234 ms] Motor: OFF
[6234 ms] Door State: OPENED
```

## Features

- **Debounced buttons** (50 ms) prevent switch bounce
- **Command priority**: STOP > OPEN > CLOSE
- **Visual feedback** with 4 LEDs showing real-time status
- **ISR-driven timers** for accurate 5-second open/close cycles
- **State machine** with 4 states: CLOSED, OPENING, OPENED, CLOSING
