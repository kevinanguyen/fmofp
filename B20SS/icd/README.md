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

**v1.3** adds the weather radar partition. The plant gains the weather (`PLANT_WEATHER`: the storm cells and the stratiform area of `weather_state_manager.py`, which the plant moves and ages, once a second), relayed to the radar as `RDR_WX_WEATHER`. The radar (RT 9, the weather radar's subaddresses) renders what its antenna would see from it exactly as `reflectivity_simulator.py` did, sweeps the circle over 1.2 s, and publishes what the scan found: `RDR_WX_SCAN` (the summary) and the `RDR_WX_PRODUCTS` block (VIL and precipitation points, storm cells, precipitation areas, wind shear events and turbulence cells of one scan), which replaces the `RDR_WX_VIL` and `RDR_WX_PRECIP` streams. With it comes the first **block payload layout**: a header and record sections, generated for both sides like any message (section 8).

**v1.4** adds the terrain-following radar. The plant gains the terrain (`fmofp_plant.terrain`: a fixed function of position whose profile north of home is the one `tfr_radar.py` invented) and publishes the elevation profile ahead of the aircraft as the `PLANT_TERRAIN` block, ten times a second. The bus controller relays it to the radar as `RDR_TFR_TERRAIN`: the first **block relay**, and the first **receive block** (`dir="R"`, `layout_of` the block it copies). The radar also receives the aircraft state (`RDR_TFR_SENSORS`) and publishes `RDR_TFR_GUIDANCE` (the clearance assessment and the first path command, every 100 ms) and the `RDR_TFR_ELEVATION` block (the assessed profile and the path commands), its payload now defined. The flight control computer takes its terrain clearance from the radar (`FCS_TERRAIN`, relayed from the guidance) instead of the plant, so `PLANT_SENSORS` loses `terrain_clearance`.

**v1.5** puts the first two displays on the bus (`B20SS/fmofp_display`). The radar display (RT 15) receives the weather and terrain-following radars' status, scan summary and guidance by relay (`DSP_RDR_WX_STATUS`, `DSP_RDR_WX_SCAN`, `DSP_RDR_TFR_STATUS`, `DSP_RDR_TFR_GUIDANCE`) and their products as receive blocks (`DSP_RDR_WX_PRODUCTS`, `DSP_RDR_TFR_ELEVATION`), and it is the radars' control panel: the radar commands move from the MFD's never-implemented `MFD_RDR_*_CMD` to `DSP_RDR_*_CMD` on RT 15. The PFD (RT 11) receives the FMS's flight data, status and attitude status by relay (`PFD_FLIGHT_DATA`, `PFD_FMS_STATUS`, `PFD_ATTITUDE_STATUS`). The display processes are Python terminals feeding the unchanged PyQt6 display classes; `cmd_seq` 0 becomes "no command" so a display that has not commanded anything commands nothing (section 5).

**v1.6** puts the MFD (RT 13) on the bus (`fmofp_display mfd`). Its radar page mirrors the radar display's pictures: the same relays (`MFD_RDR_WX_STATUS`, `MFD_RDR_WX_SCAN`, `MFD_RDR_TFR_STATUS`, `MFD_RDR_TFR_GUIDANCE`) and receive blocks (`MFD_RDR_WX_PRODUCTS`, `MFD_RDR_TFR_ELEVATION`) on the same subaddresses; its systems page reads the flight computer's GCAS alerts and envelope exceedances (`MFD_GCAS_ALERTS`, `MFD_PERF_STATUS`) and its mission page the FMS flight data (`MFD_FLIGHT_DATA`). It commands nothing. The pages whose producers are not on the bus yet (engine, communications, mission) show what the Python showed with those services not running, until those terminals exist.

## 2. Decisions this draft is built on

| Decision | From | Effect in the ICD |
|---|---|---|
| One process per subsystem | Alee | 23 subsystem and display terminals at RT 1–23, plus the plant at RT 27 from v1.1 (RT 0 reserved, 24–26 and 28–30 spare, 31 broadcast) |
| The simulated aircraft is test equipment, not flight software | v1.1 | The plant is its own terminal (RT 27). It publishes what sensors would measure; the flight partitions never contain simulation or random noise. Each quantity moves to its real producer as that subsystem is ported |
| The GPS *measurement* is simulation; the GPS *solution* is flight software | v1.2 | The plant computes the pseudoranges (geometry, clock biases, noise drawn once per epoch; the geometric ranges follow the aircraft between epochs, so a fix is current whenever the controller polls it, v1.6). The navigation terminal runs the Python's weighted-least-squares solver and the ECEF-to-WGS-84 conversion unchanged, refining a first fix it acquires in closed form (Bancroft's method). The Python started its iteration from the simulation's true position, which flight software cannot know; from anywhere else the four coplanar satellites visible from, say, the Python INS's unused starting point (35.4 N, 97.4 W) make its normal equations singular. Visibility is decided where the truth is, in the plant. The plant's home is 0 N 0 E, where the Python's navigation dict started and so where its GPS fixes were made; the 12-satellite constellation serves that spot well |
| The INS is not carried | v1.2 | `Systems/nav/ins` integrated random IMU noise into an attitude nothing used: its pitch and roll reached the FMS and were overwritten 20 times a second by the FMS's own values. Attitude comes from the FCS; position comes from the GPS/dead-reckoning fusion |
| Dead reckoning uses the airspeed as ground speed | v1.2 | `flightManagementSystem.update_flight_data` read `velocity.get('ground_speed', airspeed)`, but the key always exists with the value 0, so the live Python's dead-reckoning rates were always zero. The port uses the formula as written, with the airspeed, since the plant has no wind. The Python also crept the heading by 0.01 degrees per cycle as "small drift"; the port uses the measured heading |
| The FMS reports the attitude it was commanded, separately from the attitude it measures | v1.2 | `fmsControl.update_attitude` wrote the limited command into the FMS flight data, where the next 50 ms cycle overwrote it. `FMS_ATTITUDE_STATUS` carries the limited command (and which limits held it); `FMS_FLIGHT_DATA` carries the measured attitude from the FCS. The Python mapped FMS flight modes onto its internal flight-control object's modes, which only changed that object's loop rate; that mapping is not carried |
| The weather is test equipment; the radar's sensor model is flight software | v1.3 | In the Python the weather lived in a singleton that the radar's reflectivity simulator aged and moved whenever it rendered, and the simulator invented a cell or an area when none existed. The plant now owns the cells and the area, applies the same lifecycle and wind rules (its random draws included) and publishes them; the radar renders the reflectivity volume from them bin for bin as `_render_storm_cells`, `_render_stratiform_areas` and `_add_ground_clutter` did, and processes that. A scan latches the truth when it starts and sweeps six degrees of azimuth per frame, so one scan takes 60 frames |
| The radar's texture is repeatable | v1.3 | The Python drew `random.uniform(-2, 2)` into every stratiform bin and `10 + 5 * random()` into every clutter bin. The radar draws the same amplitudes from a hash of the scan number and the bin, so the same weather gives the same words; the oracle tests turn the texture off and patch the Python's draws to their midpoints |
| Products are where the radar measured them | v1.3 | The point products carry the azimuth and range of their column. The Python jittered every VIL point by up to ±10° and ±20 % of range "to avoid clustering", scaled the picture to 40 nm for the display, dropped points within half a mile of the aircraft, and invented up to five points by shifting indices when fewer remained. The port keeps the half-mile exclusion and the Python's selection of at most 20 VIL and 15 precipitation points (continuing points first, then the strongest clusters), measured in the Python's 40 nm display frame so the choice is the same; the jitter and the invented points are gone. Where the Python kept a random 500 of more candidates, the port keeps an even spread |
| The storm cell tracker keeps the cells it finds | v1.3 | `stormCellTracking.StormCellTracker` detected cells correctly, then deleted every new cell in the same call that created it (its "remove unmatched" pass never marked the cells it had just added), so `get_active_cells()` was always empty and no storm cell ever reached a display. The port marks new cells as matched. Its vertical development rate is measured over the scan interval; the Python divided by the wall-clock gap between two of its own statements |
| Processor geometry is the array's | v1.3 | `weather_processor`'s wind shear and turbulence processors defaulted to a 100 km range for the 200 km array, halving every position and altitude they reported; the port uses the true geometry (one km per range bin), and the oracle tests give the Python that range. The EDR proxy still falls with the cube root of the range in metres, so turbulence cells only ever appear within a few kilometres: the formula is kept as written |
| PRECIPITATION mode is defined | v1.3 | The Python logged "unknown mode" for it and answered data requests with a `KeyError`. The port gives it the mapping pattern and the precipitation products (points and areas). SURVEILLANCE produces VIL, precipitation points and storm cells; MAPPING precipitation points and areas; WINDSHEAR and TURBULENCE their own products, as the Python's `_update_radar_state` and request handlers did |
| One block per scan | v1.3 | All of a scan's products travel in one block (`RDR_WX_PRODUCTS`) so a consumer draws one consistent picture, and the five radars share RT 9's thirty subaddresses without running out. Its segment subaddress is polled every other frame (25 Hz since v1.6; every frame before): an 18-segment block arrives in 0.72 s, inside the 1.2 s scan |
| The terrain is a property of the world; the radar measures it | v1.4 | `tfr_radar.py` invented its terrain: 1000 + 500 sin(d/1000) + 200 sin(d/300) m plus 50 m of Gaussian noise at each of 100 samples over 10 km, regenerated whenever the radar entered search or track and never moving with the aircraft. The plant now owns the terrain as a fixed function of position (`fmofp_plant.terrain`: that profile when flying north from home, rolling with the easting elsewhere, two 35 m ripples in place of the noise) and publishes the profile ahead of the aircraft along its heading (`PLANT_TERRAIN`), which the bus controller relays to the radar segment by segment: the first block relay. The radar assesses what it is given, as a radar would |
| The radar assesses the real aircraft | v1.4 | `tfr_radar._send_elevation_data` assessed every profile against a fixed 1000 m ("FMS would supply real value") and `PathOptimiser.update_speed()` was never called, so the path was always planned at 250 m/s. The port receives the aircraft state (`RDR_TFR_SENSORS`): the altitude is the sensed pressure altitude and the speed the sensed airspeed (floored at 50 m/s as `update_speed` did; 250 m/s when the airspeed is invalid). The path optimiser's fixed `MIN_CLEARANCE_M` (300 m) becomes the commanded set clearance (`RDR_TFR_MODE_CMD.set_clearance`, 1000 ft until commanded); the oracle tests patch the Python's constant to the same value |
| Ground collision avoidance takes its terrain clearance from the radar | v1.4 | `groundCollisionAvoidanceSys._get_tfr_clearance` imported `get_tfr_processor` from `tfr_processor.py`, which defines no such function, so the Python's GCAS never had a clearance and its TERRAIN alert could never fire. The port relays `RDR_TFR_GUIDANCE` to the FCS as `FCS_TERRAIN`; the clearance is the minimum over the profile ahead (the processor's `min_clearance_ft`, the only clearance in feet the GCAS could have read), so TERRAIN warns of terrain ahead above the flight path, and there is no alert while the radar provides nothing, as the Python's `None` would have had it. `PLANT_SENSORS.terrain_clearance`, the stand-in since v1.1, is gone |
| Every operating mode produces the same products | v1.4 | The Python ran the clearance manager and the path optimiser on every elevation profile it sent whatever its mode; the mode only gated requests. The port does the same and labels the products with the mode. The elevation block is republished when a new profile has been assessed and at least 25 frames (0.5 s; 15 until v1.6) have passed since the last one, so its ten segments, polled at 25 Hz, always arrive whole; the guidance message carries the latest assessment every frame |
| Terrain warnings keep the Python's rule | v1.4 | HIGH_TERRAIN above 1500 m, else STEEP_TERRAIN more than 300 m from 1000 m. The 1000 m reference was the mean of the invented terrain; the rule is kept as written and recorded as the enum's definition |
| A display is a terminal fed by relays; the display classes are unchanged | v1.5 | Each display process (`fmofp_display radar`, `fmofp_display pfd`, `fmofp_display mfd` from v1.6) runs a Python terminal in a thread beside Qt. The bus controller relays what the display needs into its receive subaddresses and receive blocks, and a feed turns the words into the dictionaries the PyQt6 display classes under `FMOFP/Interfaces` always drew (the coordinator's stores for the weather display, the terrain display's data dictionary, `get_flight_data()` and `get_tactical_status()` for the PFD), so `radar_to_display_bridge.py` and the in-process messaging stack are not needed. The displays' own validity rule (stale after 2 s, any SSM but NORMAL is missing data) does the rest. A Python display answers a 50 Hz bus from beside a paint loop: give the controller a 20 ms reply timeout and run it without `--strict` when displays are on the bus |
| The radar display is the radars' control panel | v1.5 | One command subaddress has one commander, or two senders' sequence numbers would fight. v1 had proposed the MFD's radar page as the commander (`MFD_RDR_*_CMD`); the MFD is the Python's radar-mode menu but a later display, and the radar display is where the radars' pictures and status are. Its `DSP_RDR_*_CMD` sit on RT 15 SA 20-24 and are relayed into RT 9 SA 1-5 every 100 ms; the MFD, when ported, mirrors the pictures |
| Products are drawn where the radar measured them, at the radar's range | v1.5 | The weather display's items get true east and north in nautical miles from the radar's azimuth and range, and its range scale follows the range in effect (`RDR_WX_SCAN.range_km`), so the range rings read what the radar covers. The Python had scaled every product into a fixed 40 nm picture. The display's wind shear and turbulence overlays assume a 100 nm picture of their own: that is the display's quirk, left alone |
| `cmd_seq` 0 is no command | v1.5 | A transmit subaddress nobody has loaded reads as zeros and is relayed like anything else; before this rule a display joining the bus would have put every radar into STANDBY. Senders count 1 to 255 and wrap to 1 |
| The MFD mirrors the radars' pictures and reads the flight computer by relay | v1.6 | The Python MFD's radar page embedded the same display classes as the radar display and fed them from the same stores, so on the bus it receives the same relays and blocks on the same subaddresses, from a second copy the controller makes. That copy is not free: a block relayed twice costs a second 32-word transfer every time its segment is polled, and with the MFD listed the busiest minor frame reached 10.35 ms of the controller's 10 ms (50%) budget, so the controller refused the schedule. The two radar product blocks therefore move at 25 Hz (a weather block in 0.72 s, within the 1.2 s scan; a terrain block in 0.4 s, republished every 0.5 s) and the controller lays each message in the least-loaded phase of its period, which keeps the two blocks' segments in different frames; the busiest frame is then 8.1 ms (41%) with eight terminals. A third display that wants the pictures should get them by broadcast rather than a third copy (section 12). Its terrain picture read the TFR's clearance manager in-process (`_read_tfr_clearance`); that is now the relayed guidance, reported as 0 m with the level UNKNOWN while the radar provides none, because the terrain display compares the clearance with its bands and cannot draw the None the Python handed it (the radar window makes the same substitution). It commands nothing (the radar display is the control panel), and its settings page's theme and display-type choices stay local to its own process |
| A page whose producer is not on the bus yet shows what the Python showed without it | v1.6 | The MFD's systems, communications and mission pages import the engine, comms and mission services and read their dictionaries; when a service was not running the Python drew zeros and dashes. The display process does not import those services: the feed answers the same getters with the bus data it has (GCAS alerts, exceedances, the position) and an empty dictionary for the rest, until the engine (RT 3), communications (RT 2) and mission (RT 6) terminals exist. The MFD's engine page is unreachable in the frozen Python (not in its menu); its engine rows are on the systems page |
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

**Added in v1.3:**

| Field | Now | Taken from |
|---|---|---|
| Weather truth | `PLANT_WEATHER`: up to 8 storm cells (azimuth, range, radius, peak dBZ, which vertical profile) and one stratiform area (azimuth, range, width, depth, dBZ), once a second | `weather_state_manager.StormCell` / `StratiformArea` and `reflectivity_simulator._initialize_weather_state` |
| Scan summary | `RDR_WX_SCAN`: scan id, mode, pattern size, range and tilt in effect, strongest return, the counts of every product before the block's limits, the block version carrying them | New; what `weather_radar.get_status()` and the log lines said |
| VIL point | azimuth, range, value (kg/m2, the Python's formula), intensity, layer count, show flag | `vil_data_generator_sync.generate_vil_data_objects` |
| Precipitation point | azimuth, range, rate (mm/h), intensity, type, show flag | `precipitation_data_generator_sync.generate_precipitation_data_objects` |
| Storm cell | id, centroid azimuth and range, mean dBZ, size (half a mile per bin), intensity, azimuth and range rates, cell-top estimate and its rate | `stormCellTracking.StormCell` |
| Precipitation area | centroid, mean dBZ, rate, intensity, type | `precipitation_analysis.PrecipitationData` |
| Wind shear event | azimuth, range, shear (kt/nm), severity, microburst flag, divergence, beam height | `weather_processor.WindShearEvent` |
| Turbulence cell | azimuth, range, intensity, spectrum width, category, beam height | `weather_processor.TurbulenceCell` |
| Types and grades | `precip_type` (RAIN, SNOW, MIXED, HAIL, HEAVY_RAIN), `shear_severity` (LOW, MODERATE, SEVERE), `turbulence_category` (LIGHT to EXTREME) | The strings those modules produced |

**Added in v1.4:**

| Field | Now | Taken from |
|---|---|---|
| Terrain profile | `PLANT_TERRAIN` block: sequence, range (10,000 m), 100 elevation samples (quarter-metre, nearest first), sample i at i × range / 99 | `tfr_radar._initialize_terrain_data` (`numpy.linspace(0, 10000, 100)`) |
| Guidance | `RDR_TFR_GUIDANCE`: validity, master clearance level, mode, pull-up / high / steep flags, profile sequence, warning count, clearance under the aircraft, minimum clearance and its distance, first commanded altitude and its distance, set clearance | `tfr_processor.ClearanceManager` (`master_level`, `min_clearance_ft`), `PathOptimiser.compute_path`, `tfr_radar._check_terrain_warnings` |
| Assessed profile | `RDR_TFR_ELEVATION` section `profile`: elevation, clearance level, terrain warning per sample | `tfr_radar`'s elevation profile and terrain warning messages, `ClearanceWarning.level` |
| Path command | `RDR_TFR_ELEVATION` section `commands`: distance, altitude (half-metre), reason; the nearest 32 | `tfr_processor.AltitudeCommand` |
| Levels and reasons | `clearance_level` (CLEAR, LOW, CAUTION, PULL_UP), `terrain_warning` (HIGH_TERRAIN, STEEP_TERRAIN), `path_reason` (TERRAIN_AVOIDANCE, CLEARANCE, OBSTACLE) | The strings those modules produced; OBSTACLE was named but never produced |

**Added in v1.5:**

| Field | Now | Taken from |
|---|---|---|
| Radar display inputs | `DSP_RDR_WX_STATUS`, `DSP_RDR_WX_SCAN`, `DSP_RDR_TFR_STATUS`, `DSP_RDR_TFR_GUIDANCE` (relays) and the receive blocks `DSP_RDR_WX_PRODUCTS`, `DSP_RDR_TFR_ELEVATION` | What `weather_radar_display.py` read from the coordinator's `vil`, `precipitation`, `cells`, `windshear` and `turbulence` stores and what `mfd.py` handed `tfr_radar_display.py` (`tfr_data`, `terrain_clearance`, `system_health`) |
| Radar commands | `DSP_RDR_WX_CMD`, `DSP_RDR_TFR_CMD` (and the proposed SAR, targeting and AEWC ones) on RT 15 | `MFD_RDR_*_CMD` of v1, moved |
| PFD inputs | `PFD_FLIGHT_DATA`, `PFD_FMS_STATUS`, `PFD_ATTITUDE_STATUS` (relays) | `pfd.update_flight_data()`: `get_flight_data()`'s attitude, velocity, navigation, tactical and status groups; `get_tactical_status()`'s `envelope_warnings` and `profile_limits` |

**Added in v1.6:**

| Field | Now | Taken from |
|---|---|---|
| MFD radar inputs | `MFD_RDR_WX_STATUS`, `MFD_RDR_WX_SCAN`, `MFD_RDR_TFR_STATUS`, `MFD_RDR_TFR_GUIDANCE` (relays) and the receive blocks `MFD_RDR_WX_PRODUCTS`, `MFD_RDR_TFR_ELEVATION` | `mfd.draw_radar_page()`: the radar display classes it embeds, `_get_radar_data()` and `_read_tfr_clearance()` |
| MFD systems inputs | `MFD_GCAS_ALERTS`, `MFD_PERF_STATUS` (relays) | `mfd.draw_systems_page()`: `get_gcas().get_alerts()` (code, severity, message) and `get_performance_monitor().get_exceedances()` (parameter, value, limit) |
| MFD flight data | `MFD_FLIGHT_DATA` (relay) | `mfd.draw_mission_page()`'s position rows (`get_mission_service().get_data()['current_position']`) and the roll, pitch and vertical speed an exceedance row shows |
| Block rates | `RDR_WX_PRODUCTS`, `RDR_TFR_ELEVATION` and their relayed copies at 25 Hz (were 50 Hz); the 25 Hz rate group | The controller's bus-load check: with the MFD's copies the busiest frame reached 10.35 ms of the 10 ms budget at 50 Hz |

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

`cmd_seq` 0 is **no command** (v1.5). A sender counts 1 to 255 and wraps to 1. A transmit subaddress that nobody has loaded reads as zeros, and the BC relays those words like any others, so without this rule a display that had not yet commanded anything would have put every radar into STANDBY the moment it joined the bus.

A terminal can't start a transfer on a 1553 bus. A display that needs to command another terminal publishes the command at one of its own transmit subaddresses, and the BC copies it RT-to-RT. For example, the radar display's radar commands go from RT 15 SA 20–24 to RT 9 SA 1–5 every 100 ms.

## 6. Timing and the bus schedule

- **Minor frame:** 20 ms (50 Hz). **Major frame:** 1 s, which is 50 minor frames.
- **Rate groups:** 50, 25, 10, 5 and 1 Hz (25 Hz added in v1.6 for the radar product blocks).
  - 10 Hz messages go in every 5th minor frame.
  - 5 Hz messages go in every 10th.
  - 1 Hz messages go once per major frame.
  - The schedule table spreads them across offsets to balance the load.
- **Sync:** each minor frame starts with `SYNCHRONIZE_WITH_DATA` (mode code 17) sent to broadcast RT 31.
  - Its data word is the major frame count (bits 15–6, mod 1024) and the minor frame number (bits 5–0, 0–49).
  - Terminals run one frame of work when it arrives. All processes therefore stay in lockstep with the BC, and the BC can run in real time, as fast as possible, or one frame at a time for tests.
- **Bus load:** a minor frame may use at most **50%** of a real 1 Mbit/s bus. That's 20 µs per word plus about 12 µs of gaps per message. The UDP link is much faster, but this keeps the schedule one a real bus could carry.
- **RT-to-RT transfers** carry a producer's message to its consumers, so the data message and the message the consumer receives have the same layout. They are listed in the ICD's **relays** section (`<relay from=… to=…>`): in the minor frame where the bus controller polls `from`, it writes the same words into `to` straight away, and the consumer's receive message gets no slot of its own. A producer may feed several consumers, one relay each (at most four; `PLANT_SENSORS` goes to the FCS, the FMS and navigation). `icd_tool.py check` enforces that the two share a layout, that `from` transmits periodically and `to` receives, that a message has at most one relay feeding it, and the limit of four consumers.
- **The schedule itself** is built by the bus controller at start-up from the message rates and the relays (`B20SS/ofp/bc`): each message takes the least-loaded phase of its period (v1.6; before that the phases rotated in definition order), and the estimated bus load of every minor frame is checked against the 50% budget. Only the terminals actually present are scheduled; a relay to an absent consumer is skipped.

**Retries.** A transaction that draws no reply within the controller's response timeout is sent once more before it counts as a no-response and the controller moves on (v1.5; MIL-STD-1553 practice). A terminal that is gone costs two timeouts per transaction, so the controller's reply timeout stays short; a Python terminal that missed one deadline answers the retry.

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
- Each transmit command to the segment subaddress returns the next segment; the terminal moves on when the bus controller has read the one loaded, and after the last it starts again from segment 0 so a consumer that joined late still gets them all. A new payload gets the next version (1 to 255, never 0: an unloaded subaddress reads as version 0 and means nothing) and starts again from segment 0.
- The descriptor and the segment subaddress are polled at the block's rate like any periodic message (`PLANT_TERRAIN`: every frame; `RDR_WX_PRODUCTS` and `RDR_TFR_ELEVATION`: every other frame, 25 Hz).
- The largest block is 256 × 31 = 7,936 words.
- A consumer accepts a block only when it has every segment of one version and the CRC matches. Otherwise it keeps the previous block, which goes stale normally.

**Block relays.** A relay may connect two block streams. The receiving block has `dir="R"` and is `layout_of` the transmitted one; in the frame where the bus controller polls the producer's descriptor or segment subaddress, it writes the same words into the consumer's, so the consumer reassembles the block exactly as any receiver does. A receive block is never polled. On each side the framing helpers do the work: `bus::BlockPublisher` / `fmofp_bus.block.BlockPublisher` serve a producer's block from its two transmit subaddresses, and `bus::BlockCollector` / `fmofp_bus.block.BlockCollector` collect a consumer's from its two receive subaddresses. The first is `PLANT_TERRAIN` → `RDR_TFR_TERRAIN` (v1.4); the displays will receive the radars' products the same way.

**The payload** of a block whose layout the ICD defines (a `<header>` and `<section>` elements in the XML) is its header words, then one count word per section, then every section's records in order; a section's count is at most its `max`. The generated code (`fmofp/icd/blocks.hpp`, `fmofp_bus/icd/blocks.py`) encodes and decodes the whole payload and refuses one whose counts and length disagree; `bus/block.hpp` and `fmofp_bus/block.py` do the descriptor and segment framing on each side. `ICD_TABLES.md` lists the layouts under *Block payloads*. Blocks still marked `TBD-v2` have no layout yet and are not scheduled.

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

Done since v1: the C++ and Python code generators (`gen-cpp`, `gen-py`), the relays section, the bus-load check (in the bus controller), the first partition's inputs and outputs (v1.1), the navigation and flight management partitions' inputs and outputs, the PFD's flight data (`FMS_FLIGHT_DATA`), thrust and gear commands (v1.2), block payload layouts and the weather radar's inputs and products (v1.3), the terrain-following radar's inputs and products, block relays and receive blocks, terrain clearance from the radar (v1.4), the radar display and the PFD on the bus, the radar commands' home (v1.5), the MFD on the bus (v1.6).

- Payload layouts for the three remaining radar block streams (targeting tracks, AEWC sector, SAR image), as those radars are ported.
- The remaining displays as terminals: EICAS, TSD, SMS and HUD, each with the relays it needs, as their sources are ported. The MFD's engine, communications and mission rows fill in when the engine (RT 3), communications (RT 2) and mission (RT 6) terminals exist.
- Broadcast relays: one RT-to-RT transfer to address 31 that every display terminal receives, with a program-wide band of broadcast receive subaddresses, before a third display wants the radar pictures. Each unicast copy is a full transfer on the bus; the MFD's copies pushed the schedule past the 50% budget until the product blocks dropped to 25 Hz.
- Retiring `radar_to_display_bridge.py` and the in-process messaging stack from the Python once no display reads them.
- Periodic data messages for each remaining subsystem and display input: EICAS, TSD fused tracks, SMS stores, HUD. These come from the display and subsystem inventories, and are added partition by partition as the port reaches them.
- Engine data (N1, EGT, fuel flow) from RT 3, which `FCS_PERF_STATUS` needs for fuel efficiency.
- A radar altimeter in the plant, if a consumer ever needs the height over the ground directly below rather than the TFR's look-ahead clearance.
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
| 1.3-draft | 2026-10-11 | Weather radar partition: `PLANT_WEATHER` → `RDR_WX_WEATHER`; `RDR_WX_SCAN`; the `RDR_WX_PRODUCTS` block with the first payload layout (header + sections), replacing `RDR_WX_VIL` and `RDR_WX_PRECIP`. Enums `precip_type`, `shear_severity`, `turbulence_category` |
| 1.2-draft | 2026-10-11 | Navigation and flight management partitions: `PLANT_GPS`; `NAV_SENSORS`, `NAV_GPS`, `NAV_SOLUTION`; `FMS_ATTITUDE_IN`, `FMS_SENSORS`, `FMS_NAV_SOLUTION`, `FMS_FLIGHT_DATA` (replacing the `FMS_DATA` block), `FMS_FLIGHT_MODE_CMD`, `FMS_CONTROL_CMD`/`STATUS`, `FMS_ENERGY`; `FMS_STATUS`, `FMS_ATTITUDE_STATUS`, `FMS_NAV_UPDATE_STATUS` extended. Double-word fields; relays to several consumers. Enums `fms_flight_mode`, `countermeasures_state`, `targeting_state`, `weapons_state` |
| 1.4-draft | 2026-10-25 | Terrain-following radar partition: the `PLANT_TERRAIN` block → `RDR_TFR_TERRAIN` (the first block relay and receive block); `RDR_TFR_SENSORS`; `RDR_TFR_GUIDANCE` → `FCS_TERRAIN`; the `RDR_TFR_ELEVATION` block's payload. `PLANT_SENSORS` loses `terrain_clearance`; `FCS_GCAS_ALERTS.terrain_clearance` signed. Enums `clearance_level`, `terrain_warning`, `path_reason` |
| 1.5-draft | 2026-10-25 | The radar display (RT 15) and the PFD (RT 11) on the bus: `DSP_RDR_WX_STATUS`, `DSP_RDR_WX_SCAN`, `DSP_RDR_TFR_STATUS`, `DSP_RDR_TFR_GUIDANCE`, the receive blocks `DSP_RDR_WX_PRODUCTS` and `DSP_RDR_TFR_ELEVATION`; `DSP_RDR_*_CMD` replacing `MFD_RDR_*_CMD`; `PFD_FLIGHT_DATA`, `PFD_FMS_STATUS`, `PFD_ATTITUDE_STATUS`. `cmd_seq` 0 is no command |
| 1.6-draft | 2026-10-25 | The MFD (RT 13) on the bus: `MFD_RDR_WX_STATUS`, `MFD_RDR_WX_SCAN`, `MFD_RDR_TFR_STATUS`, `MFD_RDR_TFR_GUIDANCE`, the receive blocks `MFD_RDR_WX_PRODUCTS` and `MFD_RDR_TFR_ELEVATION`, `MFD_GCAS_ALERTS`, `MFD_PERF_STATUS`, `MFD_FLIGHT_DATA`; nine relays. The radar product blocks and their copies at 25 Hz (a new rate group); the controller assigns each message the least-loaded phase of its period |
