# Sensor firmware structure

## Normal program

Run `make` from the repository root. The normal firmware starts in main.c,
initializes HAL/clocks/GPIO/UART/I2C, then repeatedly calls:

```c
SHT31_Reading reading = {0};
SHT31_Status sensor_status = SHT31_Read(&hi2c1, &reading);
report_sample(counter, sensor_status, &reading);
```

- Core/Src/main.c owns the heartbeat, attempt counter, UART formatting and
  I2C1 peripheral recovery. It displays Fahrenheit, matching the previous normal
  loop; the reading also contains Celsius.
- Core/Inc/sht31.h defines the reading and status interface.
- Core/Src/sht31.c sends the existing 0x24/0x00 command to address 0x44,
  waits 20 ms, reads six bytes and validates both CRCs before converting.
- Measurements use integer hundredths. 2345 is 23.45; there is no printf-float
  linker requirement. Division truncates fractional hundredths.
- On failure, the driver leaves its output unchanged. Main prints an error,
  attempts peripheral reset after a transfer error, and retries next iteration.
  The counter includes failures.
- Recovery resets the STM32 I2C1 peripheral only. It does not guarantee recovery
  from a slave holding a line low or repair loose/unpowered hardware.
- UART transfer failure retains the existing Error_Handler stop policy.
- Sampling is a blocking superloop; the one-second delay follows transaction
  work, so it is not an exact 1 Hz schedule. No RTOS is involved.

Current wiring is PB8/SCL and PB9/SDA at 100 kHz, sensor powered at 3.3 V with
common ground and suitable pull-ups. USART2 remains PA2/PA3, 115200 8N1.

## Bring-up diagnostics

Core/Src/sensor_test.c is preserved unchanged as the original diagnostic
reference. It still does pin-level checks, scans addresses and prints Celsius.
It runs instead of the normal loop when SENSOR_TEST is enabled.

Use a separate output directory so normal/test builds never reuse each other's
main object:

```sh
make SENSOR_TEST=1 BUILD_DIR=build_sensor_test
```

Flash the ELF from the directory for the mode you actually built.
A default `make` does not enable SENSOR_TEST. Neither build automatically flashes.

## Host tests without the board

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -Itests/stubs -ICore/Inc Core/Src/sht31.c tests/test_sht31.c \
  -o /tmp/test_sht31
/tmp/test_sht31
```

The stub HAL header is used only by this command, not the embedded build.
Tests exercise the actual driver with a published CRC vector, conversion
endpoints, byte order, corrupt frames, invalid inputs and simulated HAL errors.
They verify command bytes, wait/read order and that failed transfers don't
replace output values. They do not prove electrical or sensor accuracy.

## CubeMX and learning notes

Custom main.c code stays within USER CODE sections. The generated hardware
initialization is retained.
The Makefile now explicitly lists sht31.c and supports SENSOR_TEST=1.
If CubeMX regenerates the Makefile, check that those custom additions survive
and re-add them if necessary.

C concepts in this refactor:
- A struct groups one reading's temperature/humidity fields.
- An enum names mutually exclusive outcomes (OK, TX/RX/CRC failure).
- Passing &reading lets a function fill the caller's object.
- static helper functions are private to a .c file.
- The header declares the interface; the .c file defines it; main calls it.
- CRC must pass before bytes become accepted measurements.

This refactor reuses the existing sensor test's protocol/CRC and existing
application conversions. Hardware behavior must be retested after flashing.
Keep the previously working commit as a reference while validating the normal
path. No ten-minute soak/recovery or calibrated-accuracy claim is made here.

