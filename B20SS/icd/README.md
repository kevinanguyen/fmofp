# FMOFP MIL-STD-1553B Interface Control Document (ICD)

**Version 1.2 — DRAFT** · 2026-10-11 · Owner: Kevin Nguyen · Drafted by Claude for Kevin Nguyen

This is the contract for every message on the simulated 1553B bus. The C++ flight partitions and the Python displays both build from it, and nothing else may define a bus message.

| File | What it is |
|---|---|
| `fmofp_icd.xml` | The ICD itself: the single source of truth. Edit this. |
| `ICD_TABLES.md` | Every terminal, message, field, enum and mode code, generated from the XML. Don't edit by hand. |
| `README.md` | This file: the rules that apply to every message (word format, timing, transport, validity). |
| `tools/icd_tool.py` | Checks the XML, regenerates `ICD_TABLES.md`, and generates the C++ message code (`gen-cpp`, run by the `B20SS/ofp` build) and the Python message code (`gen-py`, checked in under `B20SS/fmofp_bus/icd/`). Standard-library Python. |

After any edit to the XML:

```bash
py B20SS/icd/tools/icd_tool.py render
```

`render` runs every check first and refuses to write the tables if anything fails. Among other things, it rejects:
- a value that doesn't fit its bit field
- a bit left undefined
- two messages on the same RT, subaddress and direction
- a legacy message type that isn't mapped anywhere

## 1. Scope of v1

Alee asked for v1 to be drafted from `FMOFP/rtAddressConfig.xml` and `FMOFP/MIL_STD_1553B/message_schemas.py`. All 89 message types in `MESSAGE_SCHEMAS` are mapped to a new message, a block stream, a mode code, or an explicit retirement. The full list is at the end of `ICD_TABLES.md`.

v1 does **not** yet define the data that never crossed the bus. Today that data reaches the displays through `radar_to_display_bridge.py` and direct Python calls:
- radar product payloads
- the flight data the PFD draws
- EICAS engine, fuel, hydraulic and electrical data
- TSD tracks and SMS stores

v2 adds it, based on an inventory of what each display reads and what the subsystems exchange. The only parts still marked as coming later are those block payloads. Messages that still need confirming against the displays are marked **PROPOSED**.

**v1.1** adds what the first ported partition, flight control, needs: a *plant* terminal (RT 27) that publishes the sensed flight state (`PLANT_SENSORS`), its delivery to the FCS (`FCS_SENSORS`), and the FCS's outputs (`FCS_GCAS_ALERTS`, `FCS_PERF_STATUS`). It also adds the **relays** section: the RT-to-RT transfers the bus controller performs, which is how a producer's data reaches its consumers and how the MFD's radar commands reach the radar. From v1.1 the ICD is maintained in this fork by Kevin.

**v1.2** adds the navigation and flight management partitions. The plant gains a GPS receiver front end (`PLANT_GPS`: the pseudoranges to the Python's 12-satellite constellation, once a second) and a position that moves. The navigation terminal (RT 7) receives the sensors and the pseudoranges by relay (`NAV_SENSORS`, `NAV_GPS`), solves the fix and fuses it with dead reckoning, and publishes `NAV_SOLUTION`. The FMS (RT 12) receives the FCS attitude, the sensors and the navigation solution by relay (`FMS_ATTITUDE_IN`, `FMS_SENSORS`, `FMS_NAV_SOLUTION`) and publishes what `get_flight_data()` gave the displays as `FMS_FLIGHT_DATA`, which replaces the `FMS_DATA` block. New commands: `FMS_FLIGHT_MODE_CMD` (the five flight modes) and `FMS_CONTROL_CMD` (thrust and gear). `FMS_STATUS`, `FMS_ATTITUDE_STATUS` and `FMS_NAV_UPDATE_STATUS` grew to report what the Python returned (tactical systems, the limited attitude and rates, the applied targets), and `FMS_ENERGY` carries the energy-maneuverability numbers. Two mechanisms came with it: **double-word fields** (32 bits, for latitude, longitude and pseudoranges) and relays with **several consumers** per producer.

## 2. Decisions this draft is built on

| Decision | From | Effect in the ICD |
|---|---|---|
| One process per subsystem | Alee | 23 subsystem and display terminals at RT 1–23, plus the plant at RT 27 from v1.1 (RT 0 reserved, 24–26 and 28–30 spare, 31 broadcast) |
| The simulated aircraft is test equipment, not flight software | v1.1 | The plant is its own terminal (RT 27). It publishes what sensors would measure; the flight partitions never contain simulation or random noise. Each quantity moves to its real producer as that subsystem is ported |
| The GPS *measurement* is simulation; the GPS *solution* is flight software | v1.2 | The plant computes the pseudoranges (geometry, clock biases, noise drawn once per epoch). The navigation terminal runs the Python's weighted-least-squares solver and the ECEF-to-WGS-84 conversion unchanged, refining a first fix it acquires in closed form (Bancroft's method). The Python started its iteration from the simulation's true position, which flight software cannot know; from anywhere else the four coplanar satellites visible from, say, the Python INS's unused starting point (35.4 N, 97.4 W) make its normal equations singular. Visibility is decided where the truth is, in the plant. The plant's home is 0 N 0 E, where the Python's navigation dict started and so where its GPS fixes were made; the 12-satellite constellation serves that spot well |
| The INS is not carried | v1.2 | `Systems/nav/ins` integrated random IMU noise into an attitude nothing used: its pitch and roll reached the FMS and were overwritten 20 times a second by the FMS's own values. Attitude comes from the FCS; position comes from the GPS/dead-reckoning fusion |
| Dead reckoning uses the airspeed as ground speed | v1.2 | `flightManagementSystem.update_flight_data` read `velocity.get('ground_speed', airspeed)`, but the key always exists with the value 0, so the live Python's dead-reckoning rates were always zero. The port uses the formula as written, with the airspeed, since the plant has no wind. The Python also crept the heading by 0.01 degrees per cycle as "small drift"; the port uses the measured heading |
| The FMS reports the attitude it was commanded, separately from the attitude it measures | v1.2 | `fmsControl.update_attitude` wrote the limited command into the FMS flight data, where the next 50 ms cycle overwrote it. `FMS_ATTITUDE_STATUS` carries the limited command (and which limits held it); `FMS_FLIGHT_DATA` carries the measured attitude from the FCS. The Python mapped FMS flight modes onto its internal flight-control object's modes, which only changed that object's loop rate; that mapping is not carried |
| BIT merges into avionics; FM fitness and SW configuration fold into another terminal | Alee | All three run in the avionics process at RT 1. FM fitness and SW configuration went there because both are health and maintenance functions; Alee can move them |
| Bus controller is its own process and the master clock | Alee | Every minor frame starts with a broadcast sync (mode code 17); terminals run their frame on receipt |
| One process per display | Alee | Each display needs its own RT, because one address is one terminal. RT 11 goes to the PFD; MFD, EICAS, radar display, TSD, SMS and HUD get RT 13–18 |
| Keep the current RT addresses | Alee | RT 0–10 and 12 unchanged. MIL-STD-1553B doesn't assign addresses to types of equipment (each program's ICD does), so there is no more conventional set to move to |
| Big-endian on the socket | Alee | Every word, header included |
| UDP on 127.0.0.1, port base 51000 | Alee | Section 7 |
| One machine-readable file generates both sides | Alee | XML, because Python reads it with the standard library and it matches the existing config files |
| Freeze the Python code until the conversion is finished | Alee | The port is measured against upstream `main` at `0ab9236` (2026-10-08). Nothing in it changes while it is being ported |
| Legacy `WEATHER` mode: keep it unless it's unnecessary | Alee | Dropped as unnecessary. No radar implements it, nothing sends it, and it only existed in the old lookup tables, where it clashed with FAILURE |
| Undefined values and units: full creative liberty | Alee | Defined from the code that actually uses each one (table below) |
| Heading and yaw are true | Alee | Every heading and yaw field is relative to true north |
| Radars stay one terminal | This draft | RT 9 with each radar at its old subaddress 1–5, because radar management is one subsystem today. Splitting the five radars into separate terminals would still fit on the bus (27 of 31 addresses) if that's ever wanted |
| Keep existing subaddresses | This draft | Each display's status and command keep its old SA (PFD 11 … SMS 16; HUD gets 17). FCS keeps SA 10, FMS keeps SA 15 |

**Where the previously undefined values came from (revision 3):**

| Field | Now | Taken from |
|---|---|---|
| Weather radar `scan_type` | Replaced by `range_km` (10–200 km) and `tilt` (0–90°) | `rmConfig.xml` weather limits; `WeatherRadar.set_range()` takes km |
| TFR `scan_type` | Replaced by `set_clearance` (30–5000 ft) | `rmConfig.xml` TFR `altitude_range`, read as feet |
| SAR `image_type`, `resolution` | `image_type` dropped (the SAR mode already selects it); `resolution` 0.1–10 m in 0.01 m steps | `sar_radar.py` (1.0 m per pixel) and `rmConfig.xml` |
| Targeting `track_type` | `track_filter`: ALL, HOSTILE, FRIENDLY, UNKNOWN | Threat classes in `radar_data_fusion.py` |
| AEWC `sector_type`, `sector_parameters` | `azimuth_start`, `azimuth_end`, `elevation` in degrees | The sector-scan request in `aewc_radar_messages.py` |
| FCS control surface and value | `surface` (ailerons, elevator, rudder, flaps, spoilers); `position` in % of travel | `fmsControl.set_control_surfaces()`; replaces the old handler's −1…+1 scaling |
| FCS surface status | All five surface positions every cycle, with validity | Same |
| FMS attitude command | Adds roll, pitch and yaw rates, plus "present" bits so a field is applied only when sent | `fmsControl` UPDATE_ATTITUDE |
| FMS navigation update `update_type`, `parameter_count` | Autopilot targets: altitude (ft), airspeed (kt), true heading, vertical speed (ft/min), each with a present bit | `fmsControl` SET_AUTOPILOT / `set_autopilot_targets()` |
| FMS `maneuver_type`, `parameter_count` | `maneuver` enum, plus `dive_angle` for DIVE | `fmsControl.execute_maneuver()` |
| Every `status_code` | `cmd_status`: NONE, ACCEPTED, COMPLETED, REJECTED_INVALID, REJECTED_STATE, FAILED, NOT_IMPLEMENTED | NOT_IMPLEMENTED covers BARREL_ROLL, which `fmsControl.py` doesn't implement |
| `command_parameter` (terminal, FMS and display commands) | Removed | Nothing in the code reads it |

**Added in v1.2:**

| Field | Now | Taken from |
|---|---|---|
| FMS flight mode | `fms_flight_mode`: NORMAL 0, COMBAT 1, STEALTH 2, TRAINING 3, EMERGENCY 4 | `fmsControl.available_modes`, numbered as `flightManagementSystem.send_fms_data` did |
| Tactical systems | `countermeasures_state`, `targeting_state`, `weapons_state` enums and a stealth bit in `FMS_STATUS` | `fmsControl._update_tactical_systems` |
| Thrust, landing gear | `FMS_CONTROL_CMD` / `FMS_CONTROL_STATUS`: thrust 0–1, throttle %, afterburner, gear UP/DOWN | `fmsControl.set_thrust`, `set_landing_gear` |
| Latitude, longitude | Double-word `sint`, LSB 180/2³¹ degrees (about 1 cm) | GPS noise is metres; 16 bits would be 600 m |
| Pseudorange | Double-word `sint`, LSB 1/16 m, 12 per `PLANT_GPS` | Ranges are about 20,000–26,000 km |
| Navigation solution | `NAV_SOLUTION`: position, fused altitude, GPS validity, satellites used, fix count and age, receiver clock bias | `fuseData.NavDataFusion` and `GPSReceiver` state |
| Energy | `FMS_ENERGY`: specific excess power, sustained and instantaneous turn rates, climb rate, energy advantage | `fmsControl.calculate_energy_maneuverability` |

## 3. Word format

Words are 16 bits. **Bit 15 is the most significant bit and bit 0 the least.** A field at `bits 11-8` is `(word >> 8) & 0xF`.

**Command word** (same layout as `mil_std_1553B.py` today):

| Bits | Field |
|---|---|
| 15–11 | RT address (31 = broadcast) |
| 10 | T/R: 1 = the terminal transmits, 0 = the terminal receives |
| 9–5 | Subaddress; 0 or 31 = mode code |
| 4–0 | Data word count (0 means 32), or the mode code number |

**Status word** (MIL-STD-1553B layout):

| Bits | Field |
|---|---|
| 15–11 | RT address |
| 10 | Message error |
| 9 | Instrumentation (always 0) |
| 8 | Service request |
| 7–5 | Reserved (0) |
| 4 | Broadcast command received |
| 3 | Busy |
| 2 | Subsystem flag |
| 1 | Dynamic bus control acceptance (always 0) |
| 0 | Terminal flag |

**Data word fields** have one of these types:

| Type | Meaning |
|---|---|
| `uint` | Unsigned integer. With an LSB, value = raw × LSB |
| `sint` | Two's-complement integer. With an LSB, value = raw × LSB (for example, angles use LSB 180/32768 degrees) |
| `enum` | Integer from a named enumeration in `ICD_TABLES.md`. Every enum is sized to fit its field; the checker enforces it |
| `bool` | One bit, 1 = true |
| `ssm` | Two-bit validity of another field (section 4) |
| `spare` | Sent as 0, ignored on receipt |

A field normally lives in one word. A **double-word** field spans two consecutive words (`word="3-4"`, bits `31-0`), most significant word first, and carries one two's-complement scaled value (`sint` with an LSB); latitude, longitude and the GPS pseudoranges use it. The generated code reads and writes both words; nothing else about the message changes.

**What is no longer on the bus:**
- strings, command names and request IDs
- UUIDs, message-type or command-type codes
- checksums computed with Python's `hash()`

A message is identified only by its RT address, subaddress and T/R bit.

If a value is outside the range its field can carry, the producer sends the nearest limit and marks the field `NO_COMPUTED_DATA`.

## 4. Validity and staleness

Fields that carry measured or computed data have a 2-bit **SSM** (sign/status matrix, the ARINC 429 BNR convention):

| SSM | Meaning |
|---|---|
| 3 | NORMAL: use the value |
| 2 | FUNCTIONAL_TEST: test data, don't fly on it |
| 1 | NO_COMPUTED_DATA: the producer has no valid value |
| 0 | FAILURE_WARNING: the producer has failed |

All-zero means failure, so an uninitialised word can never be mistaken for good data.

A consumer uses a value only when its SSM is NORMAL **and** the message is fresh:
- **C++ partitions:** a periodic message is stale after **3 missed periods**.
- **Python displays:** they keep their current rule of stale after 2.0 s (`DEFAULT_STALE_AFTER_S` in `data_validity.py`), and treat any SSM other than NORMAL exactly like missing data, drawing the amber X.

## 5. Commands and the sequence-number rule

Every command carries an 8-bit `cmd_seq`, and the sender increments it for each new command. The receiver:
- acts only when `cmd_seq` differs from the last one it processed
- reports that number back as `last_cmd_seq` in its status message
- sets `cmd_rejected` if it refused the command

Re-sending a command is therefore harmless, so the BC can send commands on a fixed schedule. That replaces request IDs, retries, timeouts and the loop-prevention middleware.

A terminal can't start a transfer on a 1553 bus. A display that needs to command another terminal publishes the command at one of its own transmit subaddresses, and the BC copies it RT-to-RT. For example, the MFD's radar commands go from RT 13 SA 20–24 to RT 9 SA 1–5 every 100 ms.

## 6. Timing and the bus schedule

- **Minor frame:** 20 ms (50 Hz). **Major frame:** 1 s, which is 50 minor frames.
- **Rate groups:** 50, 10, 5 and 1 Hz.
  - 10 Hz messages go in every 5th minor frame.
  - 5 Hz messages go in every 10th.
  - 1 Hz messages go once per major frame.
  - The schedule table spreads them across offsets to balance the load.
- **Sync:** each minor frame starts with `SYNCHRONIZE_WITH_DATA` (mode code 17) sent to broadcast RT 31.
  - Its data word is the major frame count (bits 15–6, mod 1024) and the minor frame number (bits 5–0, 0–49).
  - Terminals run one frame of work when it arrives. All processes therefore stay in lockstep with the BC, and the BC can run in real time, as fast as possible, or one frame at a time for tests.
- **Bus load:** a minor frame may use at most **50%** of a real 1 Mbit/s bus. That's 20 µs per word plus about 12 µs of gaps per message. The UDP link is much faster, but this keeps the schedule one a real bus could carry.
- **RT-to-RT transfers** carry a producer's message to its consumers, so the data message and the message the consumer receives have the same layout. They are listed in the ICD's **relays** section (`<relay from=… to=…>`): in the minor frame where the bus controller polls `from`, it writes the same words into `to` straight away, and the consumer's receive message gets no slot of its own. A producer may feed several consumers, one relay each (at most four; `PLANT_SENSORS` goes to the FCS, the FMS and navigation). `icd_tool.py check` enforces that the two share a layout, that `from` transmits periodically and `to` receives, that a message has at most one relay feeding it, and the limit of four consumers.
- **The schedule itself** is built by the bus controller at start-up from the message rates and the relays (`B20SS/ofp/bc`): messages of one rate are spread across the phases of their period, and the estimated bus load of every minor frame is checked against the 50% budget. Only the terminals actually present are scheduled; a relay to an absent consumer is skipped.

## 7. Transport (the simulated wire)

UDP datagrams on **127.0.0.1 only**. UDP is used because one datagram arrives as one complete burst. The current TCP code has to handle messages split across several `recv()` calls, and many of its comments are about exactly that problem.

| Endpoint | Port |
|---|---|
| RT *n* | `port_base + n` |
| Bus controller | `port_base + 32` |
| Bus monitor | `port_base + 33` |

`port_base` defaults to **51000** and is configurable, so two simulators can run on one machine.

Each datagram is a 4-word header followed by bus words, all big-endian:

| Header word | Content |
|---|---|
| 0 | Magic `0x1553` |
| 1 | Bits 15–8: frame type (1 = BC to RT, 2 = RT to BC, 3 = monitor record). Bits 7–0: flags (bit 0 = injected parity error; the rest 0) |
| 2 | Transaction number (the BC increments it; the RT echoes it) |
| 3 | Number of bus words that follow |

**What each frame type carries:**
- **BC to RT:** the command word, plus data words if the terminal is receiving.
- **RT to BC:** the status word, plus data words if the terminal is transmitting.
- **Broadcast:** the BC sends the same datagram to every terminal. Nobody responds; each terminal sets its broadcast-received bit.

**Faults:**
- **RT-to-RT transfers are relayed by the BC.** It sends a transmit command to the source, then a receive command plus the data to the destination. The monitor records them as one transaction. This differs from a real bus, where the destination listens directly.
- **No response** within **5 ms** (configurable) is a fault for that terminal. The BC logs it and can read the terminal's status with mode code 2.
- **Fault injection is done by the BC, not the transport.** When the parity flag is set, the terminal discards the message, doesn't respond, and sets message-error in its next status word, as a real terminal would.

**Monitoring:** the BC sends every completed transaction (command word, data words, status word) to the monitor port as frame type 3. The Python recorder (SQLite) listens there instead of being called from flight code.

## 8. Block transfers (data over 32 words)

Each block stream has a **descriptor** subaddress and a **segment** subaddress, both transmit.

**Descriptor** (4 words):

| Word | Content |
|---|---|
| 0 | Bits 15–8: `block_version`, which increments whenever new data is ready. Bits 7–0: spare |
| 1 | Payload length in words |
| 2 | Number of segments |
| 3 | CRC-16/CCITT-FALSE of the payload (big-endian bytes) |

**Segment** (32 words):

| Word | Content |
|---|---|
| 0 | Bits 15–8: `block_version`. Bits 7–0: segment index |
| 1–31 | Payload; the last segment is zero-padded |

**How a block moves:**
- Each transmit command to the segment subaddress returns the next segment. The terminal starts again from segment 0 when `block_version` changes.
- The largest block is 256 × 31 = 7,936 words.
- A consumer accepts a block only when it has every segment of one version and the CRC matches. Otherwise it keeps the previous block, which goes stale normally.

## 9. Startup and the version handshake

Word 3 of every terminal's `TERM_STATUS` holds the ICD version it was built against. At startup the BC reads every terminal's `TERM_STATUS`:
- A **different major version** means the terminal is left out of the schedule and reported as incompatible.
- A **different minor version** produces a warning.

Changing a layout bumps the major version; adding something bumps the minor. While the ICD is a DRAFT and nothing is deployed against it, a layout may still grow between minor versions (as the FMS status messages did in 1.2); the major version starts counting at the first released version.

## 10. Problems found in today's bus code

I ran the real codec from `main` @ `0db2621` on test messages. That's `MetadataCodec.encode_metadata` / `decode_metadata`, which `BC_msg.py`, `Messaging.py`, `RT_socket.py`, `RT_msg.py` and the display metadata decoder all call.

| Test | Result |
|---|---|
| Any radar operating mode: SURVEILLANCE (10), PRECIPITATION (14), TERRAIN_FOLLOWING (23), STRIPMAP (30), TARGET_TRACK (41), AEWC_SEARCH (50) | **Encoding fails** with `ValueError: New bits value N exceeds the capacity of the bit range 0-2`. The modes aren't cut off; the message can't be built at all |
| Modes 0–7 (STANDBY, NORMAL, …) | Encoding "succeeds" but writes the mode over the low bits of the message-type code, because the schema is applied while that is the only word. `weather_radarStatusResponse` + STANDBY decodes as `display_data_request`; + NORMAL decodes as `weather_radarModeChangeRequest` with mode MAINTENANCE |
| INITIALIZING (−1) | The result can't be decoded: `Unknown command type code: 0` |
| FAILURE and legacy WEATHER | Both are 6, and the reverse map returns WEATHER, so a failure reads back as WEATHER |
| No mode at all | Message types still don't survive the round trip. The encoder uses a hardcoded table, but the decoder numbers types in `command_registry.xml` order. `weather_radarStatusRequest` came back as `weather_radarPrecipitationRequest` |
| VIL response without a VIL payload | The encoder crashes: `UnboundLocalError` on `vil_data` |
| Checksum | Built from Python's `hash()`, which is randomised per process, so another process can't check it |
| Format on the wire | Words are `'0'`/`'1'` text inside JSON over TCP, and every schema has `data_word_count: None` |

This ICD removes each cause:
- Each terminal gets its own mode enum, sized to fit its field.
- Every field has a fixed word and bit position.
- There are no type codes on the bus.
- INITIALIZING becomes 15.
- The legacy WEATHER value is dropped (open question 3 below).
- Blocks use a CRC-16 instead of `hash()`.
- Words travel as binary.

**Not verified:** how often live traffic reaches these paths. The displays get most of their data through `radar_to_display_bridge.py` rather than the bus.

**Decision (Alee, 2026-10-10):** leave these in the Python as they are. The bus is being replaced, and the Python code is frozen until the conversion is finished.

## 11. Questions for Alee

All answered 2026-10-10:

| # | Question | Answer | Applied |
|---|---|---|---|
| 1 | PFD at RT 11, other displays at RT 13–18? | Yes | No change needed |
| 2 | Which subsystems are their own terminals? | BIT merges into avionics; FM fitness and SW configuration fold into another terminal | All three in RT 1 (avionics); RT 24–26 freed. He didn't name a home for FM fitness and SW configuration, so they went with BIT; move them if you prefer |
| 3 | Bus controller: its own process, or inside the FMS? | Its own process | Already the design |
| 4 | UDP on 127.0.0.1, port base 51000? | Yes | No change needed |
| 5 | Which MSYS2 environment do interns use? | UCRT64 | Same as the authoring machine |
| 6 | Is the Python code still changing? | Freeze everything until the conversion is finished | Port baseline is upstream `0ab9236` |
| 7 | Fix the section 10 problems in Python? | No, leave them | Section 10 |
| 8 | Legacy `WEATHER` mode? | Keep it unless it's unnecessary | Dropped as unnecessary (section 2) |
| 9 | Undefined values and units? | Full creative liberty | Defined from the code that uses them (section 2) |
| 10 | Heading: true or magnetic? | True | Every heading and yaw field |

## 12. Backlog

Done since v1: the C++ and Python code generators (`gen-cpp`, `gen-py`), the relays section, the bus-load check (in the bus controller), the first partition's inputs and outputs (v1.1), the navigation and flight management partitions' inputs and outputs, the PFD's flight data (`FMS_FLIGHT_DATA`), thrust and gear commands (v1.2).

- Payload layouts for the six radar block streams.
- Periodic data messages for each remaining subsystem and display input: EICAS, TSD fused tracks, SMS stores, HUD. These come from the display and subsystem inventories, and are added partition by partition as the port reaches them.
- Engine data (N1, EGT, fuel flow) from RT 3, which `FCS_PERF_STATUS` needs for fuel efficiency; terrain clearance from the radar, which then replaces the plant's.
- Detailed BIT results per terminal.
- `SET_CONTROL_SURFACES` in `fmsControl.process_command()` overlaps with `FCS_SURFACE_CMD`; the FCS owns surface commands and the FMS does not forward them.
- Waypoints (`flightManagementSystem.add_waypoint` / `activate_waypoint`): nothing drew them, so they wait for the display that will.
- A wind model in the plant, after which ground speed and track stop being the airspeed and heading.

## Revision history

| Version | Date | Change |
|---|---|---|
| 1.0-draft | 2026-10-04 | First draft from `rtAddressConfig.xml` and `message_schemas.py` |
| 1.0-draft.2 | 2026-10-10 | Alee's answers 1–3: display addresses accepted; BIT, FM fitness and SW configuration moved into RT 1; bus controller confirmed as its own process; RT 24–26 freed |
| 1.0-draft.3 | 2026-10-10 | Alee's answers 4–10: UDP accepted; Python frozen at `0ab9236`; codec problems left as they are; WEATHER dropped; every undefined value defined from the code; heading true. Unused `command_parameter` words removed |
| 1.1-draft | 2026-10-10 | Flight control partition: plant terminal RT 27, `PLANT_SENSORS` → `FCS_SENSORS`, `FCS_GCAS_ALERTS`, `FCS_PERF_STATUS`; the relays section (including the MFD radar commands). Enums `gear_position`, `gcas_status`. Owner: Kevin |
| 1.2-draft | 2026-10-11 | Navigation and flight management partitions: `PLANT_GPS`; `NAV_SENSORS`, `NAV_GPS`, `NAV_SOLUTION`; `FMS_ATTITUDE_IN`, `FMS_SENSORS`, `FMS_NAV_SOLUTION`, `FMS_FLIGHT_DATA` (replacing the `FMS_DATA` block), `FMS_FLIGHT_MODE_CMD`, `FMS_CONTROL_CMD`/`STATUS`, `FMS_ENERGY`; `FMS_STATUS`, `FMS_ATTITUDE_STATUS`, `FMS_NAV_UPDATE_STATUS` extended. Double-word fields; relays to several consumers. Enums `fms_flight_mode`, `countermeasures_state`, `targeting_state`, `weapons_state` |
