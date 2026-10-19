# Driver regression

From the project root in PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tests/drivers/run.ps1
```

The script compiles the production 3508, BMI088 middleware, BMI088 driver and
attitude C files into a Windows executable with HAL/RTOS stubs. This environment
provides LCC under MATLAB R2024b; pass `-Compiler <path-to-lcc64.exe>` to select
another installation. Object files and the executable stay in `.build`.

Tests exercise encoder arithmetic, PID behavior, startup auto-enable for IDs 1-4,
zero output until a new target, cancellation before first feedback, explicit fault recovery,
bus routing, timeouts, FIFO saturation, bus-off recovery, complete SPI transfers,
failure publication, timebase wrap, asynchronous calibration and attitude validity.
They also exercise MIT Set/Send separation and header limits, grouped per-bus
transmission, pre-scheduler initialization and task ownership migration through
the unified BMI088Read sampling/calibration/attitude entry point.
Mock I/O also asserts that no SPI/CAN call occurs with interrupts masked.
`check_config.py` verifies the CubeMX settings against generated source.

These tests do not run the STM32 instruction set or reproduce peripheral timing,
interrupt priorities, task scheduling, real stack usage or hardware behavior.
