# STM32 Room Telemetry

A FreeRTOS-based environmental monitor on an STM32F401RE (Nucleo-64). It
samples temperature and humidity from an SHT31 every second, shows them on an
SSD1306 OLED, streams them over UART to a host that logs and plots them, and
accepts commands while it keeps measuring.

```
Sample 12: T=71.81 F RH=54.62 % CRC=OK
Sample 13: T=71.84 F RH=54.32 % CRC=OK ALERT
```

## Features

- **SHT31 driver** (`sht31.c`): single-shot measurement over I2C with
  CRC-8 validation; a corrupted frame is reported, never displayed.
- **SSD1306 OLED driver** (`ssd1306.c`), written from the datasheet:
  init sequence, 1 KB framebuffer, 5x7 font, page-wise transfer.
- **FreeRTOS architecture**: four tasks, a mailbox queue, an ISR-fed byte
  queue, two mutexes and event flags (details below).
- **UART command shell**: `help`, `status`, `alert <F>`, `alert off`, `hang`.
- **Temperature alert** shown on the UART stream and the OLED.
- **Task-level watchdog**: the hardware watchdog is fed only while _every_
  task keeps checking in; the reset cause is reported on the next boot.
- **Fault recovery**: I2C peripheral reset after bus errors; the OLED is
  re-initialised automatically if it stops responding.
- **Host tooling** (`tools/telemetry.py`): log readings to CSV, plot to PNG.

## Hardware

| Part                      | Notes                                                         |
| ------------------------- | ------------------------------------------------------------- |
| NUCLEO-F401RE             | 84 MHz Cortex-M4, on-board ST-Link (flash + virtual COM port) |
| SHT31-D breakout          | I2C address 0x44, on-board pull-ups                           |
| SSD1306 0.96" 128x64 OLED | I2C address 0x3C                                              |

Both devices share one I2C bus (I2C1, 100 kHz):

| Signal | Nucleo pin | SHT31 | OLED |
| ------ | ---------- | ----- | ---- |
| SCL    | D15 / PB8  | SCL   | SCL  |
| SDA    | D14 / PB9  | SDA   | SDA  |
| 3.3 V  | 3V3        | VIN   | VCC  |
| GND    | GND        | GND   | GND  |

UART: USART2 (PA2/PA3) at 115200 8N1, carried over the ST-Link USB cable.

## Architecture

```
                 ┌──────────────┐  xQueueOverwrite   ┌──────────────┐
  SHT31 ──I2C──► │  sensorTask  │ ─── sampleQueue ──► │  outputTask  │ ──I2C──► OLED
                 │  Normal      │   (length 1)        │  BelowNormal │ ──UART─► host
                 │  every 1 s   │                     └──────────────┘
                 └──────────────┘
  keyboard ──UART──► RX ISR ── rxQueue ──► commandTask (BelowNormal) ──UART─► host
                     (bytes only)          status / alert / hang

  i2cMutex:  sensorTask <-> outputTask      (shared I2C1 bus)
  uartMutex: outputTask <-> commandTask     (shared USART2)
  wdgFlags:  every task checks in ──► watchdogTask (AboveNormal) ──► IWDG
```

| Task           | Priority    | Job                                              |
| -------------- | ----------- | ------------------------------------------------ |
| `sensorTask`   | Normal      | Measure on a fixed schedule; publish each result |
| `outputTask`   | BelowNormal | Print to UART and draw on the OLED               |
| `commandTask`  | BelowNormal | Parse typed lines and act on them                |
| `watchdogTask` | AboveNormal | Feed the IWDG only if all tasks checked in       |

### Design decisions

- **Separate sensor and output tasks.** A slow or failing display must not
  delay a measurement. The sensor task has higher priority and uses
  `osDelayUntil`, so the sampling period does not stretch with output work.
- **Mailbox, not a FIFO.** `sampleQueue` holds one sample and the sensor
  overwrites it. If output falls behind, the display shows the newest reading
  instead of working through a backlog, and the sensor never blocks on it.
  The cost is that intermediate samples can be skipped. That's acceptable for
  a live display; a logging-to-flash design would use a deeper queue.
- **Mutexes with priority inheritance** for the shared I2C bus and UART. If
  the low-priority output task holds the bus when the sensor task needs it,
  it is temporarily boosted so it releases quickly (avoids priority
  inversion).
- **ISR does almost nothing.** The UART receive interrupt only pushes each
  byte into `rxQueue` and re-arms; parsing happens in the command task. The
  interrupt priority (6) is numerically above
  `configMAX_SYSCALL_INTERRUPT_PRIORITY` (5), as FreeRTOS requires for an ISR
  that calls its API.
- **Watchdog per task, not per board.** Feeding the IWDG from one loop would
  miss a single stuck task. Here each task sets a bit in an event-flags group;
  the watchdog task feeds the hardware only when all bits arrive within 2 s,
  otherwise it names the missing task and lets the hardware reset the chip.
- **Small shared state without locks where it's safe.** The alert threshold
  and the "user is typing" flag are single aligned 32-bit values, which the
  Cortex-M4 reads and writes atomically. The multi-word latest-sample copy
  used by `status` is guarded by a critical section.

## Measurements

Taken on the board with OpenOCD/GDB and host-side timestamps:

| Metric                          | Result                                                                                                                      |
| ------------------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| Sampling period                 | 0.985 s, jitter about 1 ms (offset is HSI oscillator tolerance, about ±1%). The old superloop gave about 1.13 s with drift. |
| Peak stack use (of 2048 B each) | sensor ~216 B, output ~592 B, command ~688 B                                                                                |
| FreeRTOS heap free              | ~8.4 KB of 15 KB (before adding the watchdog task)                                                                          |
| Watchdog                        | No reset over a 17 s healthy run (hardware timeout ~5 s). Use `hang` to exercise the failure path.                          |

## Build and flash

Requires `arm-none-eabi-gcc`, `make` and OpenOCD.

```sh
make -j8
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/EnvironmentalMonitor.elf verify reset exit"
```

## Use it

Open the serial console (macOS; the port number may differ):

```sh
ls /dev/cu.usbmodem*
screen /dev/cu.usbmodem3103 115200     # quit: Ctrl-A, K, Y
```

| Command     | Effect                                                       |
| ----------- | ------------------------------------------------------------ |
| `help`      | List commands                                                |
| `status`    | Latest reading, alert setting, OLED health                   |
| `alert 80`  | Flag readings above 80 °F (UART and OLED)                    |
| `alert off` | Disable the alert                                            |
| `hang`      | Test: freeze the command task; the watchdog resets the board |

Sample lines pause while you type, so a command isn't broken up by output.

### Log and plot on the host

Only one program can open the serial port, so quit `screen` first.

```sh
python3 tools/telemetry.py log          # writes data/telemetry-<date>-<time>.csv
python3 -m pip install --user matplotlib  # once, for plotting
python3 tools/telemetry.py plot data/telemetry-<...>.csv
```

Logging uses only the Python standard library. For long runs on a laptop,
keep the Mac awake: `caffeinate -i python3 tools/telemetry.py log`.

## Tests

Host-side unit tests for the SHT31 driver (CRC vector, conversions, byte
order, corrupt frames, mocked HAL failures) run without the board:

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/stubs -ICore/Inc Core/Src/sht31.c tests/test_sht31.c -o /tmp/test_sht31
/tmp/test_sht31
```

A bring-up diagnostic firmware (bus scan, line-level checks, raw reads) builds
with `make SENSOR_TEST=1 BUILD_DIR=build_sensor_test`. See
[tests/README.md](tests/README.md).

## Debugging notes

Problems found during bring-up, and how:

- **Sensor never ACKed (I2C timeout `0x20`).** A diagnostic build put weak
  pull-downs on SCL/SDA and read them back: SDA saw the sensor's pull-up, SCL
  didn't. The root cause was a broken SCL connection (unsoldered header pins
  pushed through the holes). The fix was soldering, and moving I2C1 to
  PB8/PB9.
- **OLED lit up for a test, then went blank, and UART output stopped.**
  Reading memory over SWD showed the HAL tick frozen at 2212 ms and
  `BASEPRI = 0x50`. FreeRTOS tasks had been created without starting the
  scheduler, which masks interrupts until `osKernelStart()`, so the TIM2
  timebase never ticked and the first `HAL_Delay` hung forever.
- **Stack sizing.** CubeMX's default 512 B task stack was below the measured
  ~568 B peak once `snprintf` and the 129-byte OLED packet were on the stack.
  Stacks were raised to 2 KB and checked with the `0xA5` fill pattern.

## Project layout

```
Core/Src/main.c         tasks, queues, mutexes, command shell, watchdog
Core/Src/sht31.c        SHT31 driver (CRC-checked)
Core/Src/ssd1306.c      SSD1306 OLED driver + font
Core/Src/sensor_test.c  bring-up diagnostics (SENSOR_TEST builds only)
tests/                  host unit tests for the SHT31 driver
tools/telemetry.py      host logger / plotter
EnvironmentalMonitor.ioc  CubeMX project
```

Custom code lives in CubeMX `USER CODE` sections, and the USART2 interrupt
and watchdog are set up by hand. If "USART2 global interrupt" is ever enabled
in CubeMX, remove the hand-written `USART2_IRQHandler` and NVIC setup to avoid
duplicates.

## Limitations

- I2C transfers are blocking; the sensor can wait up to about 100 ms for the
  bus while the OLED updates. DMA or interrupt-driven I2C would remove this.
- The SHT31 transaction holds the I2C bus during its 20 ms conversion wait.
- Readings are not calibrated against a reference instrument.
- The OLED font is uppercase only.
