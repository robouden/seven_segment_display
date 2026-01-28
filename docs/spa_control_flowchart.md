# Spa Control Firmware Flowchart

Generated from `spa_control.ino`

---

## System Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         SPA CONTROL SYSTEM                               │
│                                                                          │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐          │
│  │ Temp     │    │ Control  │    │ Display  │    │ Relay    │          │
│  │ Sensor   │───▶│ Logic    │───▶│ Update   │───▶│ Outputs  │          │
│  │ (ADC)    │    │          │    │          │    │          │          │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘          │
│       │                                                │                │
│       ▼                                                ▼                │
│  ┌──────────┐                                    ┌──────────┐          │
│  │ Switch   │                                    │ Status   │          │
│  │ Inputs   │                                    │ LEDs     │          │
│  └──────────┘                                    └──────────┘          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 1. Initialization Flow (setup → initializeSystem)

```mermaid
flowchart TD
    A[Power On / Reset] --> B[setup]
    B --> C[initializeSystem]

    subgraph INIT["initializeSystem()"]
        C --> D[Initialize State Variables]
        D --> D1[currentTemp = 0]
        D1 --> D2[setpoint = 38.0°C]
        D2 --> D3[heaterOn = false]
        D3 --> D4[autoMode = true]
        D4 --> D5[mode = MODE_NORMAL]

        D5 --> E[Clear Display Buffer]
        E --> E1["digitCodes[0..2] = CHAR_BLANK"]

        E1 --> F[Configure GPIO]
        F --> F1["DDRB: DATA_PIN, CLOCK_PIN = OUTPUT"]
        F1 --> F2["DDRD: LATCH_PIN = OUTPUT"]
        F2 --> F3["Set idle state: all HIGH"]

        F3 --> G[Configure Inputs]
        G --> G1["DDRD: Switch pins = INPUT"]
        G1 --> G2["PORTD: Enable pull-ups"]

        G2 --> H[Configure Relay Outputs]
        H --> H1["DDRD: HEATER_RELAY, PUMP_RELAY = OUTPUT"]
        H1 --> H2["PORTD: Both relays OFF"]

        H2 --> I[Configure ADC]
        I --> I1["ADMUX: AVcc reference, ADC0"]
        I1 --> I2["ADCSRA: Enable, prescaler 64"]

        I2 --> J[blankDisplay]
        J --> K[Read Initial Temperature]
        K --> L{temp > 0?}
        L -->|Yes| M[lastGoodTemp = currentTemp]
        L -->|No| N[Keep default 25°C]
        M --> O[Ready for Main Loop]
        N --> O
    end
```

---

## 2. Main Loop (loop)

```mermaid
flowchart TD
    A[loop] --> B[currentTime = millis]

    B --> C{"currentTime - lastSensorRead<br/>≥ SENSOR_READ_INTERVAL<br/>(500ms)?"}

    C -->|Yes| D[lastSensorRead = currentTime]
    D --> E[readSensors]
    E --> F[processTemperature]
    F --> G[controlTemperature]
    G --> H[updateDisplay]
    H --> I[updateLEDs]
    I --> J[refreshDisplay]

    C -->|No| J

    J --> A

    style D fill:#e1f5fe
    style E fill:#e1f5fe
    style F fill:#e1f5fe
    style G fill:#e1f5fe
    style H fill:#e1f5fe
    style I fill:#e1f5fe
    style J fill:#fff3e0
```

---

## 3. Sensor Reading (readSensors)

```mermaid
flowchart TD
    A[readSensors] --> B[tempReading = readTemperatureSensor]

    B --> C{"tempReading < -10°C<br/>OR > 100°C?"}

    C -->|Yes Invalid| D[sensorErrorCount++]
    D --> E{"sensorErrorCount ≥<br/>SENSOR_ERROR_THRESHOLD (3)?"}
    E -->|Yes| F["enterErrorState(1)<br/>Sensor Fault"]
    E -->|No| G[Continue with last good value]

    C -->|No Valid| H[currentTemp = tempReading]
    H --> I[lastGoodTemp = tempReading]
    I --> J[sensorErrorCount = 0]
    J --> K{mode == MODE_ERROR?}
    K -->|Yes| L[exitErrorState]
    K -->|No| M[Continue]
    L --> M

    F --> N[Read Switch Inputs]
    G --> N
    M --> N

    N --> O["pumpSwitch = !(PIND & PUMP_SWITCH_PIN)"]
    O --> P["airSwitch = !(PIND & AIR_SWITCH_PIN)"]
    P --> Q["autoSwitch = !(PIND & AUTO_SWITCH_PIN)"]

    Q --> R[Update State]
    R --> R1[pumpOn = pumpSwitch]
    R1 --> R2[airOn = airSwitch]
    R2 --> R3[autoMode = autoSwitch]
    R3 --> S[Return]
```

---

## 4. Temperature Sensor Reading (readTemperatureSensor)

```mermaid
flowchart TD
    A[readTemperatureSensor] --> B[adcSum = 0]
    B --> C[i = 0]

    C --> D{"i < ADC_SAMPLES (8)?"}
    D -->|Yes| E["ADMUX: Select ADC0"]
    E --> F["ADCSRA: Start conversion"]
    F --> G{"Conversion<br/>complete?"}
    G -->|No| G
    G -->|Yes| H[adcSum += ADC]
    H --> I[i++]
    I --> D

    D -->|No| J[adcAvg = adcSum / ADC_SAMPLES]

    J --> K{SENSOR_TYPE_NTC_10K<br/>defined?}

    K -->|Yes NTC| L{"adcAvg < 10 OR<br/>adcAvg > 1013?"}
    L -->|Yes| M[return -999.0 Invalid]
    L -->|No| N["resistance = Rpullup × (adcAvg / (1023 - adcAvg))"]
    N --> O["steinhart = ln(R/R25) / Beta + 1/Tref"]
    O --> P["temperature = 1/steinhart - 273.15"]

    K -->|No Linear| Q["voltage_mv = adcAvg × 5000 / 1024"]
    Q --> R["temperature = (voltage_mv - 500) / 10"]

    P --> S["temperature = (temp × SCALE) + OFFSET"]
    R --> S
    M --> T[Return]
    S --> T
```

---

## 5. Temperature Processing (processTemperature)

```mermaid
flowchart TD
    A[processTemperature] --> B["Apply Exponential Smoothing<br/>alpha = 0.3"]

    B --> C["filteredTemp = 0.3 × currentTemp<br/>+ 0.7 × filteredTemp"]
    C --> D[currentTemp = filteredTemp]

    D --> E{"currentTemp > MAX_TEMP_SAFE<br/>(45°C)?"}

    E -->|Yes DANGER| F[heaterOn = false]
    F --> G["PORTD &= ~HEATER_RELAY<br/>Immediate OFF"]
    G --> H["enterErrorState(2)<br/>Over Temperature"]

    E -->|No Safe| I[Return]
    H --> I
```

---

## 6. Temperature Control (controlTemperature)

```mermaid
flowchart TD
    A[controlTemperature] --> B{"mode == MODE_ERROR<br/>OR !autoMode?"}

    B -->|Yes Skip| C{mode == MODE_ERROR?}
    C -->|Yes| D[heaterOn = false]
    C -->|No| E[Return - Manual Mode]
    D --> E

    B -->|No Auto Mode| F["tempLow = setpoint - 0.5°C<br/>tempHigh = setpoint + 0.5°C"]

    F --> G{"currentTemp < tempLow<br/>(below 37.5°C)?"}

    G -->|Yes Cold| H[heaterOn = true]

    G -->|No| I{"currentTemp > tempHigh<br/>(above 38.5°C)?"}

    I -->|Yes Hot| J[heaterOn = false]

    I -->|No In Band| K[Maintain Current State]

    H --> L[Apply Relay Outputs]
    J --> L
    K --> L

    L --> M{heaterOn?}
    M -->|Yes| N["PORTD |= HEATER_RELAY"]
    M -->|No| O["PORTD &= ~HEATER_RELAY"]

    N --> P{pumpOn?}
    O --> P
    P -->|Yes| Q["PORTD |= PUMP_RELAY"]
    P -->|No| R["PORTD &= ~PUMP_RELAY"]

    Q --> S[Return]
    R --> S
```

---

## 7. Display Update (updateDisplay)

```mermaid
flowchart TD
    A[updateDisplay] --> B{mode == MODE_ERROR?}

    B -->|Yes| C[Return - Error already displayed]

    B -->|No| D["setDisplayNumber(currentTemp, 1)"]
    D --> E[Display shows temperature<br/>with 1 decimal place]
    E --> F[Return]
```

---

## 8. Display Refresh Cycle (refreshDisplay)

```mermaid
flowchart TD
    A[refreshDisplay] --> B["segments = digitCodes[currentDigit]"]

    B --> C{"decimalPosition > 0 AND<br/>currentDigit == (3 - decimalPosition)?"}

    C -->|Yes| D["segments |= CHAR_DP (0x20)"]
    C -->|No| E[Continue]
    D --> E

    E --> F["digitSelect = 0b11100000 | ledStatus"]
    F --> G["cathodeBit = getCathodeBit(currentDigit)"]

    G --> H["digitSelect &= ~(1 << cathodeBit)<br/>Activate current cathode"]

    H --> I["shiftOut16(segments, digitSelect)"]

    I --> J["currentDigit = (currentDigit + 1) % 3"]

    J --> K["delayMicroseconds(5000)"]

    K --> L[Return]

    subgraph CATHODE["getCathodeBit Mapping"]
        M["digit 0 → bit 5 (Left)"]
        N["digit 1 → bit 7 (Middle)"]
        O["digit 2 → bit 6 (Right)"]
    end
```

---

## 9. Shift Register Protocol (shiftOut16)

```mermaid
flowchart TD
    A["shiftOut16(segments, digitSelect)"] --> B["PORTB: DATA=HIGH, CLOCK=HIGH<br/>Idle State"]

    B --> C["DATA → LOW"]
    C --> D[Wait 16µs]
    D --> E["CLOCK → LOW"]
    E --> F[Wait 16µs]

    F --> G["Shift Out: segments byte<br/>8 bits, MSB first"]

    subgraph BYTE1["Segments Byte (8 bits)"]
        G --> G1[i = 7]
        G1 --> G2{"bit set?"}
        G2 -->|Yes| G3[DATA = HIGH]
        G2 -->|No| G4[DATA = LOW]
        G3 --> G5[Wait 2µs]
        G4 --> G5
        G5 --> G6[CLOCK = HIGH]
        G6 --> G7[Wait 40µs]
        G7 --> G8[CLOCK = LOW]
        G8 --> G9[Wait 20µs]
        G9 --> G10[i--]
        G10 --> G11{i >= 0?}
        G11 -->|Yes| G2
        G11 -->|No| H
    end

    H --> I["Shift Out: digitSelect byte<br/>8 bits, MSB first"]

    subgraph BYTE2["Digit Select Byte (8 bits)"]
        I --> I1[Same bit-bang process]
    end

    I1 --> J["PORTB: DATA=HIGH, CLOCK=HIGH"]
    J --> K[Wait 2µs]
    K --> L["LATCH → LOW"]
    L --> M[Wait 40µs]
    M --> N["LATCH → HIGH"]
    N --> O[Return]
```

---

## 10. LED Update (updateLEDs)

```mermaid
flowchart TD
    A[updateLEDs] --> B[ledStatus = 0]

    B --> C{heaterOn?}
    C -->|Yes| D["ledStatus |= (1 << 0)<br/>Heater LED"]
    C -->|No| E{autoMode?}
    D --> E

    E -->|Yes| F["ledStatus |= (1 << 1)<br/>Auto LED"]
    E -->|No| G{airOn?}
    F --> G

    G -->|Yes| H["ledStatus |= (1 << 3)<br/>Air LED"]
    G -->|No| I{pumpOn?}
    H --> I

    I -->|Yes| J["ledStatus |= (1 << 4)<br/>Pump LED"]
    I -->|No| K[Return]
    J --> K
```

---

## 11. Error Handling

```mermaid
flowchart TD
    subgraph ENTER["enterErrorState(errorCode)"]
        A[enterErrorState] --> B[mode = MODE_ERROR]
        B --> C[heaterOn = false]
        C --> D["PORTD &= ~HEATER_RELAY<br/>Safety Shutdown"]
        D --> E["setDisplayError(errorCode)"]
        E --> F["Display shows 'Er1' or 'Er2'"]
        F --> G["ledStatus |= HEATER_LED<br/>Flash indicator"]
    end

    subgraph EXIT["exitErrorState()"]
        H[exitErrorState] --> I[mode = MODE_NORMAL]
        I --> J[sensorErrorCount = 0]
    end

    subgraph DISPLAY["setDisplayError(code)"]
        K[setDisplayError] --> L["digitCodes[0] = 'E'"]
        L --> M["digitCodes[1] = 'r'"]
        M --> N["digitCodes[2] = code digit"]
        N --> O[decimalPosition = 0]
    end
```

---

## State Machine Overview

```mermaid
stateDiagram-v2
    [*] --> MODE_NORMAL: Power On

    MODE_NORMAL --> MODE_NORMAL: Valid sensor reading
    MODE_NORMAL --> MODE_ERROR: 3 consecutive bad readings
    MODE_NORMAL --> MODE_ERROR: Temperature > 45°C

    MODE_ERROR --> MODE_NORMAL: Valid sensor reading
    MODE_ERROR --> MODE_ERROR: Heater forced OFF

    note right of MODE_NORMAL
        - Heater control active (if autoMode)
        - Display shows temperature
        - LEDs show status
    end note

    note right of MODE_ERROR
        - Heater forced OFF
        - Display shows "Er#"
        - Heater LED flashes
    end note
```

---

## Timing Diagram

```
Time ─────────────────────────────────────────────────────────────────────▶

Sensor Read     ┌──┐                                    ┌──┐
(every 500ms)   ┘  └────────────────────────────────────┘  └──────────────

Control Logic   ────┌──┐                                    ┌──┐
(after sensor)      └──┘                                    └──┘

Display Mux     ┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐┌┐
(continuous)    ┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└┘└

                ◀──── 5ms ────▶
                    per digit

Digit 0 (Left)  ██      ██      ██      ██      ██      ██      ██
Digit 1 (Mid)     ██      ██      ██      ██      ██      ██      ██
Digit 2 (Right)     ██      ██      ██      ██      ██      ██      ██

                ◀──────── 15ms full cycle ────────▶
                        (~67Hz refresh rate)
```

---

## Data Flow Summary

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              INPUTS                                      │
├─────────────┬─────────────┬─────────────┬─────────────┬─────────────────┤
│ Temp Sensor │ Pump Switch │ Air Switch  │ Auto Switch │                 │
│    (ADC0)   │   (PD2)     │   (PD3)     │   (PD4)     │                 │
└──────┬──────┴──────┬──────┴──────┬──────┴──────┬──────┴─────────────────┘
       │             │             │             │
       ▼             ▼             ▼             ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         STATE STRUCTURE                                  │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐     │
│  │ currentTemp │  │ pumpOn      │  │ airOn       │  │ autoMode    │     │
│  │ setpoint    │  │ heaterOn    │  │ mode        │  │ ledStatus   │     │
│  │ lastGoodTemp│  │ digitCodes[]│  │ errorCount  │  │             │     │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘     │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │
       ┌────────────────────────┼────────────────────────┐
       ▼                        ▼                        ▼
┌─────────────────┐  ┌─────────────────────┐  ┌─────────────────┐
│  HEATER RELAY   │  │   7-SEGMENT DISPLAY │  │   STATUS LEDs   │
│     (PD6)       │  │  (Shift Registers)  │  │ (Shift Regs)    │
│                 │  │                     │  │                 │
│  ON: temp < 37.5│  │  Normal: "38.5"     │  │ Bit 0: Heater   │
│  OFF: temp > 38.5│ │  Error:  "Er1"      │  │ Bit 1: Auto     │
└─────────────────┘  └─────────────────────┘  │ Bit 3: Air      │
                                              │ Bit 4: Pump     │
┌─────────────────┐                           └─────────────────┘
│   PUMP RELAY    │
│     (PD7)       │
│                 │
│  Follows switch │
└─────────────────┘
```

---

## Error Codes

| Code | Display | Meaning | Trigger |
|------|---------|---------|---------|
| 1 | Er1 | Sensor Fault | 3 consecutive readings outside -10°C to 100°C |
| 2 | Er2 | Over Temperature | Temperature exceeds 45°C |
