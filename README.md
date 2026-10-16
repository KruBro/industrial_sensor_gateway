# Industrial Sensor Gateway

An STM32 Modbus RTU **slave** (three analog sensors, one valve, an LCD) and a Linux **master**
that polls it, stores the data, raises alarms and answers an HMI. The whole system runs in the
PicSimLab simulator, connected through a virtual serial cable.

This README records what was built, how it works, how it was tested, what went wrong along the
way, and what is still open. It follows the project design document (*Industrial Sensor Gateway,
Design Document v1.0*) and lists every place where the implementation had to differ from it.

---

## 1. System overview

```
 PicSimLab (STM32F103 "Blue Pill")                    Linux host
┌──────────────────────────────────┐   virtual   ┌──────────────────────────────────────┐
│ 3 potentiometers -> ADC (PA0-2)  │   serial    │  master (poll loop)                  │
│ button -> PB0 (valve toggle)     │   cable     │    |- Modbus read / write + retries  │
│ LCD (I2C, PB6/PB7)               │             │    |- process database (mmap file)   │
│ USART1 PA9/PA10, 9600 8N1        │             │    |- alarm child process + log file │
│   Modbus RTU slave, address 0x01 │<--tnt0|tnt1-->    |- HMI Unix socket server        │
└──────────────────────────────────┘             └──────────────────────────────────────┘
                                                        ^
                                                        |  text lines over a Unix socket
                                                   hmi_client  /  nc -U  /  (optional) Node-RED
```

| Layer | Job |
|---|---|
| Field (STM32) | Reads the sensors, holds them in Modbus registers, answers requests. It never speaks first. |
| Gateway (Linux) | The only Modbus master. Polls, stores the latest values, raises alarms, serves the HMI. |
| Supervisory (HMI) | Reads values and sends valve commands through a socket. |

**Protocol:** Modbus RTU over a serial line, 9600 baud, 8 data bits, no parity, 1 stop bit.
Function codes used: **0x03** (read holding registers) and **0x05** (write single coil).

**Board used:** PicSimLab simulates a **Blue Pill (STM32F103C8T6)** at 72 MHz. The design document
mentions a Nucleo-64 (STM32F446RE) at 180 MHz; its timing numbers do not apply here.

---

## 2. Repository layout

### Linux side (this repository)

```
.
├── Makefile
├── README.md
├── common/                protocol code, no OS or HAL calls (identical on both sides)
│   ├── crc16.c/.h           CRC-16/MODBUS
│   └── modbus.c/.h          frame builders, validator, request parser
├── gateway/               the Linux master
│   ├── main.c               entry point only: parse args, open the port, run, close
│   ├── main_helper.c/.h     command line, single read, counting mode, statistics
│   ├── master.c/.h          one Modbus read transaction, status codes, retries
│   ├── serial_io.c/.h       open the port, send/receive a frame with timeouts
│   ├── poller.c/.h          timerfd + poll() loop           (M4)
│   ├── db.c/.h              process database in shared memory (M7)
│   ├── alarm.c/.h           alarm pipe, child process, log   (M8)
│   └── hmi.c/.h, hmi_client.c   HMI socket server and client (M9)
└── tests/
    ├── test_modbus.c        unit tests for common/
    └── fake_slave_test.py   master against a fake slave on a pseudo-terminal
```

### STM32 firmware (CubeIDE project)

| File | Role |
|---|---|
| `main.c` | Clock, GPIO, ADC, I2C, USART1 setup; the main loop; UART receive callbacks; frame extraction (`mbport_get_frame`); button ISR callback |
| `lcdi2c.c/.h` | HD44780 LCD driver over a PCF8574 I2C backpack (address 0x27) |
| `crc16.c/.h`, `modbus.c/.h` | The same protocol code as `common/`, plus the slave handler `mb_handle_request()` and the register table `regs[]` |
| `stm32f1xx_it.c` | Interrupt handlers, including `EXTI0_IRQHandler` and `USART1_IRQHandler` |
| `stm32f1xx_hal_msp.c` | Pin and clock setup for ADC, I2C, UART, including the USART1 interrupt enable |

**Why two copies of `modbus.c`:** the firmware copy contains the slave handler and `regs[]`, which
need `valve_state` from `main.c`. The Linux copy is protocol only. The tidy fix is to move the
handler into its own firmware file (for example `slave.c`), so both sides share one identical
`common/modbus.c`. This has not been done yet.

---

## 3. Hardware and register map

### Pins (firmware)

| Pin | Function |
|---|---|
| PA0, PA1, PA2 | ADC channels 0, 1, 2: pressure, flow, temperature |
| PB0 | Valve button (falling edge, internal pull-up, EXTI0) |
| PB6, PB7 | I2C1 SCL / SDA to the LCD backpack |
| PA9, PA10 | USART1 TX / RX (Modbus) |

The design document uses USART2 (PA2/PA3). On this board PA2 is the temperature ADC input, so
**USART1 on PA9/PA10** is used instead.

### Clocks

HSE through the PLL x9 gives 72 MHz. The ADC clock is PCLK2 / 6 (12 MHz). SysTick gives a 1 ms tick
(`HAL_GetTick`).

### Registers

| Register | Content | Scaling |
|---|---|---|
| `regs[0]` | Pressure | `adc * 4000 / 4095` = 0 to 400.0 bar, **times 10** |
| `regs[1]` | Flow | `adc * 10000 / 4095` = 0 to 1000.0 L/min, **times 10** |
| `regs[2]` | Temperature | `adc * 1500 / 4095` = 0 to 150.0 degrees C, **times 10** |
| `regs[3]` | Valve state (0 closed, 1 open) | mirror of `valve_state`, added so the master can read it |
| `regs[4..9]` | Unused, 0 | |
| coil 0 | Valve | written with FC 0x05 |

A register value of 3260 means 326.0 bar. The slave has no coil-read function, so the valve
state is mirrored into `regs[3]`; otherwise the master could not learn about a button press.

---

## 4. Modbus RTU implementation

### 4.1 Frames

All 16-bit fields are sent **high byte first**. The CRC is sent **low byte first**.

| Frame | Layout | Length |
|---|---|---|
| Read request | `addr 03 startHi startLo qtyHi qtyLo crcLo crcHi` | 8 |
| Read response | `addr 03 byteCount [regHi regLo]... crcLo crcHi` | 5 + 2n |
| Write coil request | `addr 05 coilHi coilLo valHi valLo crcLo crcHi` (ON = `FF 00`, OFF = `00 00`) | 8 |
| Write coil response | exact echo of the request | 8 |
| Exception response | `addr (function \| 0x80) code crcLo crcHi` | 5 |

Exception codes: **01** illegal function, **02** illegal data address, **03** illegal data value.

### 4.2 CRC-16/MODBUS

Start with `0xFFFF`. For each byte: XOR it into the low byte of the CRC, then do 8 rounds of
"if the lowest bit is 1, shift right and XOR with `0xA001`; otherwise just shift right". No final
XOR. Verified against an independent Python implementation and the standard check value.

| Input | CRC |
|---|---|
| *(empty)* | `0xFFFF` |
| `01` | `0x807E` (wire bytes `7E 80`) |
| `"123456789"` | `0x4B37` |
| `01 03 00 00 00 03` | `0xCB05` (wire bytes **`05 CB`**) |
| `01 03 00 00 00 0A` | `0xCDC5` (wire bytes `C5 CD`) |
| any frame with its own CRC appended | `0x0000` (the receiver's validity test) |

### 4.3 Frame examples (all CRCs verified independently)

| Meaning | Bytes |
|---|---|
| Read 3 registers from register 0 | `01 03 00 00 00 03 05 CB` |
| Valve ON | `01 05 00 00 FF 00 8C 3A` |
| Valve OFF | `01 05 00 00 00 00 CD CA` |
| Response, registers 0x0123 0x0456 0x0789 | `01 03 06 01 23 04 56 07 89 C7 D5` |
| Exception: FC 03, illegal address | `01 83 02 C0 F1` |
| Exception: FC 03, illegal value | `01 83 03 01 31` |
| Exception: FC 05, illegal address | `01 85 02 C3 51` |
| Exception: FC 05, illegal value | `01 85 03 02 91` |
| Exception: unsupported function 0x10 | `01 90 01 8D C0` |

### 4.4 Library functions (`common/`)

| Function | Does |
|---|---|
| `modbus_crc16(buf, len)` | CRC over `len` bytes. For a frame, `len` is the frame length **minus 2**. |
| `modbus_build_read_request` | Builds an 8-byte read request |
| `modbus_build_write_coil_request` | Builds an 8-byte coil write; any non-zero flag sends `FF 00` |
| `modbus_build_read_response` | Builds a response; returns 0 if the count is 0 or above 125 |
| `modbus_build_exception` | Builds a 5-byte exception; sets bit 7 itself |
| `modbus_validate_frame` | Checks length (at least 4), then the CRC. Returns `MB_OK`, `MB_ERR_SHORT` or `MB_ERR_CRC` |
| `modbus_parse_request` | Validates, then fills a `ModbusFrame`. Also `MB_ERR_FUNCTION`, `MB_ERR_LENGTH`. On an unsupported function or wrong length it still fills the address and function code, so a slave can build an exception reply. |

Rules the code follows: frames are built byte by byte, never by copying a struct (padding and
endianness would corrupt it); status codes are `int`, never `uint8_t` (negative values would turn
into large positive ones and every comparison would silently fail).

Bugs found and fixed while writing this layer: a hard-coded `buf[6]` and `buf[7]` in the validator
(it only worked on 8-byte frames), a coil flag sent as `1` instead of `0xFF00`, the missing `0x80`
bit on exception replies, `||` where `&&` was needed in the function check, and a decode loop that
read only two of three registers.

---

## 5. STM32 firmware in detail

### 5.1 The first problem: the button needed many presses

The valve is toggled in the button interrupt, and that part worked. The causes were elsewhere:

1. **A slow display.** Every LCD character went out as two nibbles. Each nibble made three
   separate I2C transmissions and a 1 ms delay, so one screen refresh took hundreds of
   milliseconds in the emulator. The state changed at once but the screen caught up late, so the
   user pressed again and flipped it back.
2. **No debounce.** One press can produce several edges; an even number cancels out.

Fixes: each nibble is **one 3-byte I2C transfer** (data, data with enable, data without enable)
and the per-nibble delay is gone; a line is redrawn **only when its text changed**; every line is
padded to 16 characters (so `OPEN` does not leave `ED` from `CLOSED`); the button callback
ignores edges within **200 ms** of the last accepted one (`HAL_GetTick`).

### 5.2 UART receive path

- USART1, 9600 8N1, interrupt driven. `HAL_UART_Receive_IT` receives one byte at a time.
- `HAL_UART_RxCpltCallback` stores the byte in a 256-byte buffer, records `HAL_GetTick()`, and
  **re-arms the receive on every path**. Forgetting the re-arm gives "works once, then silent".
- `HAL_UART_ErrorCallback` records the error and re-arms.
- The USART1 interrupt must be enabled in the NVIC (`HAL_UART_MspInit`, priority 1, below the
  button's 0) and `USART1_IRQHandler` must exist in `stm32f1xx_it.c`. Without both, no byte ever
  arrives and nothing reports an error.
- **Do not use `__HAL_UART_CLEAR_OREFLAG` in the error callback.** It reads `USART_DR`, and the
  emulated UART treats a read of an empty data register as fatal; PicSimLab crashed with
  `Read value from USART_DR while it was empty`.
- Nothing is parsed in the interrupt. The callbacks only store bytes.

### 5.3 Finding a frame: `mbport_get_frame()`

Real Modbus ends a frame after 3.5 character times of silence (about 4 ms at 9600 baud). In the
simulator bytes arrive far apart (section 7), so silence cannot be used. The firmware instead
detects a frame **by length and CRC**, as the design document's own flowchart does:

1. If 8 or more bytes are buffered, copy the first 8 out (interrupts disabled only during the copy).
2. Check the CRC. **Good:** remove those 8 bytes and return the frame.
3. **Bad:** drop **one** byte and try again. This is the resync: a single stray byte can no longer
   misalign every later frame.
4. A partial frame (fewer than 8 bytes) that has been silent for **1 second** is discarded.
5. An overflow clears the buffer.

### 5.4 The slave handler `mb_handle_request()`

| Situation | Reply |
|---|---|
| Frame too short or CRC bad | **nothing** |
| Address is neither 0x01 nor 0 (broadcast) | **nothing** |
| Unsupported function | exception 01 |
| Wrong length | exception 03 |
| FC 03, quantity outside 1 to 10 | exception 03 |
| FC 03, start + quantity above 10 | exception 02 |
| FC 03, valid | the registers |
| FC 05, coil other than 0 | exception 02 |
| FC 05, value not `FF00` or `0000` | exception 03 |
| FC 05, valid | set the valve, reply with an exact echo |
| Broadcast (address 0) | the write is carried out, **no reply** |

Everything runs in the main loop, so `regs[]` and `valve_state` need no locking beyond the byte
accesses the button interrupt makes (single bytes are atomic on this CPU).

### 5.5 Debug display

`RX_DEBUG` (a `#define`, 0 for normal use) replaces the LCD lines with receive diagnostics:
line 1 `X<count> <first 5 bytes>` is a snapshot of the last data thrown away; line 2
`N<count> <first 5 bytes>` is the live buffer. `--` marks an empty position. This is how the
missing-byte problem was investigated.

---

## 6. Test setup

### 6.1 Virtual serial cable

The final setup uses **tty0tty**, which creates linked devices `/dev/tnt0` and `/dev/tnt1`
(a null-modem pair). Earlier work used `socat` with `/tmp/ttyV0` and `/tmp/ttyV1`; the pair
behaves the same way for this purpose.

```bash
# one time
sudo dnf install gcc make git kernel-devel-$(uname -r)
git clone https://github.com/lcgamboa/tty0tty && cd tty0tty/module && make && sudo make install

# every boot
sudo modprobe tty0tty
sudo chmod 666 /dev/tnt0 /dev/tnt1
stty -F /dev/tnt1 9600 raw -echo
```

| Device | Used by |
|---|---|
| `/dev/tnt0` | PicSimLab (the STM32's USART1) |
| `/dev/tnt1` | the master, or test scripts |

### 6.2 Order of starting

1. Load the tty0tty module and set the permissions.
2. Start **one** PicSimLab with the firmware. Set its UART part to `/dev/tnt0` at 9600.
3. Check: `sudo lsof /dev/tnt0 2>/dev/null` shows exactly one `picsimlab` line.
4. Start the master on `/dev/tnt1`.

**Only one program may read a port at a time.** Each incoming byte goes to only one reader. A
second PicSimLab instance was found holding `/dev/tnt0` (launched from the IDE and never closed);
check with `pgrep -a picsimlab` before each session.

### 6.3 Sending test frames by hand

```bash
cat > ~/send.sh <<'EOS'
#!/bin/bash
gap=$1; shift
exec 3>/dev/tnt1
for b in "$@"; do printf "\x$b" >&3; sleep "$gap"; done
exec 3>&-
EOS
chmod +x ~/send.sh

~/send.sh 0.2 01 03 00 00 00 03 05 CB      # 200 ms between bytes
```

A reader that prints replies in hex:

```bash
python3 -c "
import os
fd=os.open('/dev/tnt1',os.O_RDONLY)
while True: print(os.read(fd,16).hex(' '),flush=True)"
```

---

## 7. The byte-loss investigation

### 7.1 What was observed

With the earlier `socat` setup, requests sent to the STM32 often arrived incomplete. The count of
bytes stored by the firmware depended on how fast the bytes were written:

| Gap between bytes | Bytes stored (of 8) | Error flag |
|---|---|---|
| 200 ms | 8 | none |
| 150 ms, 100 ms | 8 | none |
| 120 ms | 7 in one run (that run also had the timeout code disabled, so it is not a clean comparison) | none |
| 70 ms, 50 ms | 7 | 1 in some runs |
| 10 ms | 1 to 3 | 1 to 2 |
| one burst write | **1** (only the last byte survived) | 1 |

A single byte, or a request containing `00` bytes, arrived normally. After a reset, the first
frame often lost its first byte (`01`); later frames were fine.

### 7.2 What was ruled out

| Suspect | How it was ruled out |
|---|---|
| Baud rate | The last byte was always correct, and there were no framing errors |
| Zero bytes in the frame | A frame with three `00` bytes arrived complete at 200 ms |
| Main loop load (LCD, ADC) | The loss still happened with that work disabled |
| The frame timeout code | The loss still happened with it disabled, and happened before it existed |
| Terminal settings | Raw mode on both ends, echo off |

### 7.3 What it probably is, and what is not known

The loss depends on send speed and the error flag undercounts it (seven bytes lost, one error).
The most likely explanation is that PicSimLab's emulated UART accepts bytes in batches and the
simulated CPU reads only the last. **This is unproven**: there was no way to look inside PicSimLab.

Two things changed together and then the first-frame loss stopped: `HAL_UART_Receive_IT` was
moved before the slow LCD initialization, and the link moved from `socat` to tty0tty. Which one
fixed it was not tested separately. Afterwards, every test frame sent with 200 ms gaps produced
the expected reply **without** a dummy byte.

Later a **second PicSimLab instance** was found holding the same port. If both read it, bytes can
go to the wrong one. Whether that contributed to the losses was not checked.

The burst-send case has not been re-measured on tty0tty with a single instance.

### 7.4 Workarounds in the design

- Frame detection by **length and CRC**, not by silence (section 5.3).
- CRC **resync**: a stray or missing byte costs one frame, not every later frame.
- The master can send **byte by byte** with a delay: `make SIM_BYTE_DELAY_US=120000`. Default 0
  (whole-frame write). Real hardware should use 0.
- The master **retries** lost or damaged replies (section 8.3).
- The poll interval is a configuration value, not the design document's fixed 100 ms.

---

## 8. Linux master

### 8.1 Serial port (`serial_io`)

`open_port()` opens the port with `O_RDWR | O_NOCTTY` and configures raw mode: 9600 8N1, receiver on,
`CLOCAL`, and **software flow control explicitly off** (`IXON | IXOFF | IXANY` cleared). The design
document's snippet leaves it on, and Linux would then swallow any `0x11` or `0x13` byte.

`send_serial_frame()` writes the whole frame, or one byte at a time with a delay, and handles
partial writes and `EINTR`. `read_serial_frame()` waits with `poll()` against one total deadline
and returns **the number of bytes received**, so a short or empty result is not lost as a plain
error. It returns -1 only for real errors (including `POLLERR`, `POLLHUP`, `POLLNVAL`).

### 8.2 One read transaction (`master.c`)

`modbus_read_registers()` flushes stale input, sends the request, and reads the reply in **two
stages**: first the 5 bytes every reply has, then the rest.

- If byte 1 has bit 7 set, it is a 5-byte exception reply. It is checked and reported at once.
  Reading 11 bytes in one go would wait the full timeout for bytes that never come.
- Otherwise the byte count must equal `2 * count`, then the remaining `2 * count` bytes are read.
- Checks, in order: length, CRC, address, function, byte count. Registers are decoded big-endian.
- Both reads share one 2000 ms deadline.

Status codes: `MBM_OK`, `MBM_ERR_ARGS`, `MBM_ERR_IO`, `MBM_ERR_TIMEOUT`, `MBM_ERR_SHORT`,
`MBM_ERR_CRC`, `MBM_ERR_ADDRESS`, `MBM_ERR_FUNCTION`, `MBM_ERR_BYTECOUNT`, `MBM_ERR_EXCEPTION`.
The function never prints; the caller turns a code into a message.

### 8.3 Retries and counting mode

`modbus_read_registers_retry()` repeats the transaction up to 3 times, 100 ms apart (no pause after
the last attempt).

| Status | Retried? | Reason |
|---|---|---|
| Timeout, short reply, bad CRC, wrong byte count | yes | the reply was lost or damaged |
| Wrong address, wrong function | yes | a stray or damaged frame on a one-slave bus |
| Exception reply | **no** | the slave understood and refused; asking again gives the same answer |
| Bad arguments, serial I/O error | **no** | retrying cannot fix them |

Reads are safe to repeat. `usleep()` takes microseconds: an early version passed 100 and slept
0.2 ms instead of 100 ms.

**Counting mode** repeats the read N times and prints one summary line:

```
runs=20  ok_first_try=17  ok_after_retry=2  failed=1  attempts=24
failures:  no reply (timeout)=1
```

It exists because the link was unreliable: a single successful run proves nothing, while a rate
does. It is a measuring tool for the report, not part of the Modbus protocol.

### 8.4 Command line

```
build/master [port] [count 1-125] [slave address 0-247] [runs 1-1000]
```

`runs = 1` prints the values (`P=326.0 bar  F=355.0 L/min  T=45.0 C` when `count` is 3). `runs > 1`
runs counting mode. The exit code is 0 only if every run succeeded. Defaults: `/dev/tnt1`, 3
registers, address 1.

### 8.5 Poll loop (`poller.c`, M4)

A `timerfd` (`CLOCK_MONOTONIC`) fires every interval and `poll()` waits on it, so there is no
`sleep()` anywhere. Each tick must be `read()` (it returns a count of ticks; more than one means
ticks were missed and the extra ones are skipped, never queued). `SIGINT` and `SIGTERM` only set a
flag; the loop checks it and exits cleanly with code 0.

Points raised in review: the loop should keep running after a failed poll (stopping only for I/O
or argument errors), return non-zero when it stops because of an error, reject an interval of 0
(a zero timer value disarms the timer), and return errors instead of calling `exit()` in library
code so `main()` can clean up.

### 8.6 Non-blocking transaction (M5)

While a transaction waits for the slave, the program must not freeze: the HMI socket needs answers
in well under 50 ms, and a failing link can block a transaction for about 6 seconds with retries.
The transaction becomes a small state machine driven by `poll()` (idle, waiting for the 5-byte
head, waiting for the body, done). Deadlines replace blocking timeouts, retries move into the state
machine, and only one transaction is in flight at a time, with a small queue in front for valve
commands. The validation and decoding code from `master.c` is reused unchanged.

### 8.7 Valve command (M6)

The master builds the request with `modbus_build_write_coil_request()` and sends it through the
transaction queue. The normal reply is an exact **echo**; it must match the request. An
exception reply (function `0x85`) is reported. Retries are safe because writing the same value
twice does no harm. The firmware mirrors `valve_state` into `regs[3]`, and the master polls four
registers (pressure, flow, temperature, valve).

### 8.8 Process database (M7)

A flat struct in a file mapped with `mmap(MAP_SHARED)`, written only by the master. Latest
registers, the valve state, `device_online`, the time of the last good poll, and counters for polls,
CRC errors and failed polls.

- After a good poll: copy the registers, set the valve state from `regs[3]`, increment the poll
  count, store the time, set `device_online = 1`.
- After enough failed polls in a row (3), `device_online = 0`; the next good poll sets it back.
- The design document says the struct is 32 bytes. That is wrong. The document's own layout is
  **44** bytes on this compiler. A layout using a 64-bit timestamp and explicit padding (34 bytes of
  fields plus 6 of padding) is **40**. Use fixed-width types (`uint64_t`, not `time_t`) and a
  `static_assert` on the size, so any accidental change fails the build.
- The document says both "no locking" and "lock with `fcntl`". A reader can see pressure from one
  poll and flow from the next. A sequence counter (written before and after each update; readers
  retry when it is odd or changed) avoids that.
- `msync` only forces data to disk; other processes see `MAP_SHARED` writes at once.

### 8.9 Alarms (M8)

A child process, created with `pipe()` and `fork()`, appends alarm lines to a log file, so a slow
disk can never delay polling.

- The parent compares each value with its limit after a good poll and writes an `AlarmEvent`
  (28 bytes) into the pipe only on the **rising edge** (value was at or below the limit, now above).
  It re-arms when the value falls back.
- The write end is non-blocking. If the pipe is full, the event is dropped and counted.
- The child closes every inherited descriptor it does not need, reads whole structs, and exits
  when the pipe closes. The parent reaps it with `waitpid(WNOHANG)` on `SIGCHLD`.
- Limits are in register units (times 10): the design document's temperature limit of 120 degrees C
  is **1200**. The document gives no other limits and does not define the three severity levels;
  these are design decisions.

### 8.10 HMI socket (M9, defined)

`AF_UNIX` / `SOCK_STREAM` server inside the master's `poll()`. One line per message, ending in `\n`.

| You send | You get back |
|---|---|
| `PRESSURE` | `PRESSURE 326.0 bar` |
| `FLOW` | `FLOW 355.0 L/min` |
| `TEMP` | `TEMP 45.0 C` |
| `VALVE` | `VALVE OPEN` or `VALVE CLOSED` |
| `ALL` | `{"p":326.0,"f":355.0,"t":45.0,"valve":"OPEN","online":true}` |
| `VALVE:OPEN` / `VALVE:CLOSE` | `OK`, or an error text |
| anything else | `ERR unknown tag` |
| any plain query while the slave is offline | `OFFLINE` |

Queries are answered from the process database, with no serial round trip. The JSON for `ALL` is
built by hand (the design document allows no external libraries), using integer division by 10
and not floating point, so a locale that uses a decimal comma cannot produce invalid JSON.
Writes use `MSG_NOSIGNAL` so a client that disconnects cannot kill the master.

Optional: a Node-RED dashboard (an `inject` node feeding an `exec` node running
`echo ALL | nc -U -w 1 <socket>`, then a `json` node and gauges). Untested; not in the design
document.

---

## 9. Tests

| Suite | What it checks | Count |
|---|---|---|
| `tests/test_modbus.c` | CRC vectors (including every single-bit flip), all builders, the validator (good, corrupted, swapped CRC bytes, 5-, 8-, 11-byte frames, short buffers), the parser (eight cases) | 77 checks |
| Slave handler test (firmware logic, run on a PC) | Every row of the section 5.4 table, plus valve state after each write | 19 checks |
| `tests/fake_slave_test.py` | The master against a fake slave on a pseudo-terminal, with an exact count of requests received | 23 cases |

The fake slave covers: a normal reply, a reply arriving in pieces, 1 and 10 registers, an exception
(exactly 1 request, answered fast), a first request lost, a first reply damaged, bad CRC / wrong
address / wrong function / wrong byte count / partial / no reply (3 requests each), the byte-delay
build, counting mode (good runs, first try always lost, exceptions, bad CRC with the failure
breakdown), and rejected arguments.

```bash
make              # build/master
make test         # unit tests
make test-master  # master vs fake slave (needs python3)
make clean
```

The frame-extraction function was also tested on a PC with stand-ins for the interrupt and tick
calls (clean frame, one to three stray bytes in front, back-to-back frames, a partial frame
before and after its timeout, garbage, a corrupted frame, an overflow, an output buffer that is too
small).

**Result of the 20-run test against PicSimLab** (`build/master /dev/tnt1 3 1 20`): *to be filled in.*

---

## 10. Milestones

| # | Milestone | State |
|---|---|---|
| M1 | Serial port setup, send one request, print the bytes | done |
| M2 | Validate the reply, decode and print P, F, T | done |
| M3 | Retries, status codes, counting mode | done |
| restructure | Folder layout, `Makefile`, tests moved out of `main.c`, source split into `main`, `main_helper`, `master`, `serial_io` | done |
| M4 | Poll loop (`timerfd` + `poll()`) | done |
| M5 | Non-blocking transaction | done |
| M6 | Valve command (FC 0x05), `regs[3]` mirror | done |
| M7 | Process database | done |
| M8 | Alarms | done |
| M9 | HMI socket server, client, `ALL` for Node-RED | defined, not started |
| M10 | Clean shutdown and robustness | defined |
| M11 | Acceptance tests T01 to T10 and the report | defined |

### M10: clean shutdown

One cleanup function that every exit path goes through (no `exit()` in library code). On `SIGTERM`
or `SIGINT` the handler only sets a flag, and the main loop then stops polling, closes the serial
port and timerfd, closes the HMI sockets and removes the socket file, sets `device_online = 0` and
unmaps the database, closes the alarm pipe (the child exits on end-of-file), waits for the child,
and exits 0. Also: ignore `SIGPIPE`, check every system call, handle `EINTR`, and handle a stale
socket or database file from a crash.

Tests: `kill -TERM` gives exit code 0, no socket file, no zombie, no child left; `kill -9` followed
by a restart recovers; a run under `-fsanitize=address,undefined` or valgrind is clean; ten minutes
with repeated HMI connections keeps the descriptor count and memory constant.

### M11: acceptance tests and report

| Test | How it is run here | Note |
|---|---|---|
| T01 pressure pot to maximum | `hmi_client PRESSURE` | near 400.0; "within 200 ms" may not be reachable, so measure it |
| T02 flow pot to the middle | `hmi_client FLOW` | about 500.0 L/min |
| T03 temperature above the limit | move the pot, read the log | limit 120 C is 1200 in register units; exactly one entry |
| T04 press the valve button | `hmi_client VALVE`, look at the LCD | needs the `regs[3]` mirror |
| T05 pressure query | `echo PRESSURE \| nc -U <socket>` | also check `ALL` |
| T06 `VALVE:OPEN` | through the socket | check the LCD and the next `VALVE` query |
| T07 stop PicSimLab | `hmi_client PRESSURE` | `OFFLINE`; measure how long detection takes |
| T08 corrupt a byte | corrupt the **response** (fake slave) | see section 11 |
| T09 `SIGTERM` | `kill -TERM` | exit 0, socket file gone |
| T10 log after T03 | read the log | timestamp, tag, value, limit |

Measurements to collect: first-try success rate over at least 100 reads, the poll interval actually
achieved, time to detect an offline slave, socket response time (also during a slow poll).

---

## 11. Differences from the design document

| Design document | What was found or done |
|---|---|
| CRC of `01 03 00 00 00 03` is `C4 7E` | It is `05 CB` (checked independently) |
| `ProcessDB` is 32 bytes | 44 bytes with its own layout; 40 with a 64-bit timestamp and 6 bytes of padding |
| USART2 | USART1 on PA9/PA10, because PA2 is an ADC input |
| Nucleo-64, 180 MHz | Blue Pill in PicSimLab, 72 MHz |
| ADC with DMA | One channel at a time, polled: the emulator supports neither scan nor continuous mode |
| Wait for 8 bytes, or 3.5 character times | Length plus CRC, with resync and a 1 s timeout for partial frames |
| 100 ms poll interval, replies within 200 ms | A configuration value, measured; the simulator cannot be assumed to meet 100 ms |
| `/var/run/...`, `chmod 777 /var/run /var/log` | Configurable paths with defaults under `/tmp`; do not open permissions on system folders |
| "no locking" and "`fcntl` lock" for the database | Contradictory; a sequence counter is used or the choice is documented |
| T08: corrupt one byte of a **request** | A slave drops a request with a bad CRC silently, so the master sees only a timeout and `crc_errors` does not rise. Corrupt a **response** instead. |
| Slave supports FC 03 and FC 05 only | No coil read exists, so the valve state is mirrored into `regs[3]` |
| Valve state read as a coil | Read as a holding register |
| `NoStretch` and 100 kHz I2C | Unchanged |

---

## 12. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `multiple definition of 'main'` or `undefined reference to 'mbm_strerror'` | Stale object files after replacing sources. Run `make clean && make`. Files copied from a zip can look older than the objects. |
| No reply at all | `sudo lsof /dev/tnt0` must list exactly one `picsimlab`. Zero means it is not attached; two means a stale instance. Use `pgrep -a picsimlab`. |
| Bytes arrive split between programs | Two readers on one port. Close the Python reader or terminal before running the master. |
| Replies are wrong or garbled | Run `stty -F /dev/tnt1 9600 raw -echo`. Check that PicSimLab's UART part is at 9600. |
| `Permission denied` on `/dev/tnt1` | `sudo chmod 666 /dev/tnt0 /dev/tnt1` (lost on reboot). A udev rule makes it permanent. |
| `modprobe` fails | Secure Boot may block the unsigned module. Check `dmesg \| tail`. Missing `kernel-devel` for the running kernel stops the build. |
| `send: command not found` | `send` was a shell function, gone in a new terminal. Use `~/send.sh`. |
| Receive counter stays 0 | USART1 interrupt not enabled in the NVIC, or `USART1_IRQHandler` missing, or the firmware was not reloaded into PicSimLab. |
| PicSimLab crashes with `Read value from USART_DR while it was empty` | `__HAL_UART_CLEAR_OREFLAG` in the error callback. Remove it. |
| First frame after reset lost its first byte | Seen with the first setup. `HAL_UART_Receive_IT` must be armed before the slow LCD initialization. Cause not fully confirmed (see section 7.3). |

---

## 13. Known limitations and open items

- The cause of the simulator byte loss is **not proven**. The measurements in section 7 are
  single runs and the behavior was intermittent near 100 ms.
- The burst-send case and the 20-run success rate have not been measured on tty0tty with a single
  PicSimLab instance.
- The firmware and Linux `modbus.c` are two copies. Moving the slave handler into its own file
  would let both use one `common/modbus.c`.
- Limits and severity levels for the alarms are decisions, not values from the design document
  (except the 120 degrees C temperature limit).
- Hardware (a real RS-485 link) was never used. The byte delay and the length-based framing are
  simulator workarounds; a real board should use whole-frame writes and could use DMA with
  idle-line detection on the receive side.