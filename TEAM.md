# Team and subsystems

Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | WiFi communication, command and telemetry; system State Machine | 2303567 | CR0N14 | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion control: motors, encoders, speed control, turning | 2303576 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following and barcode decoding | 2303564 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU motion and terrain monitoring | 2303557 | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Adaptive ultrasonic scanning and obstacle profiling | 2303571 | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 | ultrasonic ECHO (in). Unplug before building with `CRASH_REPORT=1` |
| GP1 | S5 | ultrasonic TRIG (out). Unplug before building with `CRASH_REPORT=1` |
| GP2 | S4 | I2C1 SDA, GY-511 IMU |
| GP3 | S4 | I2C1 SCL, GY-511 IMU |
| GP4 | S2 | left encoder A |
| GP5 | S2 | left encoder B |
| GP6 | | |
| GP7 | | |
| GP8 | S2 | left motor M1A (PWM) |
| GP9 | S2 | left motor M1B (PWM) |
| GP10 | S2 | right motor M2A (PWM) |
| GP11 | S2 | right motor M2B (PWM) |
| GP12 | S5 | SG90 servo signal (PWM) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S2 | right encoder A |
| GP17 | S2 | right encoder B |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | S3 | left IR sensor, ADC0 |
| GP27 | S3 | centre IR sensor, ADC1 |
| GP28 | S3 | right IR sensor, ADC2 |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | S4 | IMU task | GP2, GP3 |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 | Line Following task | GP26–GP28 |
| PWM slices 4, 5 | S2 | Motion Control task | GP8–GP11 |
| PWM slice 6 | S5 | Ultrasound task | GP12 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4).
Application priorities follow the Week 6 design review RTOS task table and
are defined in one place, `app_program/app.h` (`PRI_...`), so no two tasks
pick the same level by accident. Change them only by team agreement.
Write down every task you create.

| Subsystem | Task | Priority (`app.h`) | Core | Why |
|---|---|---|---|---|
| 1 | State Machine | 10 `PRI_STATE_MACHINE` | any | mode changes and fault handling must be prompt; never blocks for long |
| 2 | Motion Control | 11 `PRI_MOTION_CONTROL` | any | 10 ms speed loop and emergency braking |
| 3 | Line Following | 12 `PRI_LINE_FOLLOWING` | any | stale steering makes the car leave the track |
| 4 | IMU | 13 `PRI_IMU` | any | 10 ms sampling; late samples add hump-height error |
| 5 | Ultrasound | 14 `PRI_ULTRASOUND` | any | obstacle response, but must not pre-empt control |
| 3 | Barcode Decoder | 15 `PRI_BARCODE` | any | route decisions tolerate some delay |
| 1 | Networking | 16 `PRI_NETWORKING` | any | telemetry and reconnects tolerate delay; must never delay control |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | | | | | |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | | | | | |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | | | | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | | | | |
| Message buffer | 12 | | | | | |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | | | | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
