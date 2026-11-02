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

**v1.7** sends the displays' data by broadcast. The radars' status, scan summary, guidance and product blocks, the flight computer's GCAS alerts and exceedances and the FMS's flight data, status and attitude status are relayed once each to address 31 (`BCAST_RDR_WX_STATUS`, `BCAST_RDR_WX_SCAN`, `BCAST_RDR_TFR_STATUS`, `BCAST_RDR_TFR_GUIDANCE`, `BCAST_RDR_WX_PRODUCTS`, `BCAST_RDR_TFR_ELEVATION`, `BCAST_FCS_GCAS_ALERTS`, `BCAST_FCS_PERF_STATUS`, `BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS`, `BCAST_FMS_ATTITUDE_STATUS`), and each display accepts the broadcast subaddresses it draws; a terminal stores a broadcast only on a subaddress it has accepted. The per-display copies of v1.5 and v1.6 are removed. Adding a display costs the bus nothing for the data it already broadcasts.

**v1.8** adds the engine management partition (RT 3, `fmofp_engine`). The plant gains the engines (`fmofp_plant.engine`: `engineControlUnit.py`'s model for each of the two engines, stepped at its 2 Hz) and publishes their measurements as `PLANT_ENGINE`, relayed to the engine computer as `ENG_ENGINE_IN`. The engine computer commands them (`ENG_THRUST_CMD`, relayed to the plant as `PLANT_THRUST_CMD`) from the FMS thrust lever (`ENG_FMS_CONTROL`, a relay of `FMS_CONTROL_STATUS`), from `ENG_CMD` (thrust per engine, autothrottle, fuel transfer) and from `thrustManagement.py`'s autothrottle, which holds the airspeed of the FMS flight data heard by broadcast; it keeps `fuelControl.py`'s tanks, burning the measured fuel flow along the fuel lines, and `fuelMonitor.py`'s flags; and it publishes `ENG_STATUS`, `ENG_ENGINE_STATUS` (the measurements with the MFD engine page's assessment and the total fuel flow) and `ENG_FUEL_STATUS`. The engine and fuel status are broadcast to the MFD (`BCAST_ENG_ENGINE_STATUS`, `BCAST_ENG_FUEL_STATUS`: the engine band, subaddresses 18-19), and the flight control computer listens to the engine status for `FCS_PERF_STATUS`'s fuel efficiency. These are the first subsystems to hear broadcasts: RT 3 hears 31/25, RT 5 hears 31/18.

**v1.9** puts the EICAS (RT 14) on the bus (`fmofp_display eicas`). It adds itself to the listeners of the engine computer's engine and fuel status (`BCAST_ENG_ENGINE_STATUS`, `BCAST_ENG_FUEL_STATUS`), the flight computer's GCAS alerts and exceedances (`BCAST_FCS_GCAS_ALERTS`, `BCAST_FCS_PERF_STATUS`), the FMS's flight data and status (`BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS`) and the weather radar's scan summary (`BCAST_RDR_WX_SCAN`), which gains `max_shear`, the strongest wind shear severity of the scan, beside the turbulence category and the microburst flag it already carried, so a display that grades the weather need not reassemble the products block. Nothing else changes on the bus: the EICAS commands nothing, and every message it hears was broadcast already. Its hydraulic, electrical and environmental rows read unknown until those terminals exist.

**v1.10** adds the targeting radar and the cross-radar track fusion to the radar computer (RT 9, `fmofp_radar`), and the air traffic to the plant (`fmofp_plant.traffic`): the aircraft `targeting_radar.py` and `aewc_radar.py` invented, flown by the plant and published as what the antennas would measure of them, the `PLANT_TRAFFIC` block (relayed as `RDR_TRAFFIC`). The targeting radar keeps `targeting_radar.py`'s track table from those detections, with `target_processor.py`'s signature classification and stealth assessment, its lock and jamming flag and the track filter: `RDR_TGT_STATUS`, `RDR_TGT_MODE_CMD` (mode, filter and lock request in one command) and the `RDR_TGT_TRACKS` block, which gets its payload layout. `radar_data_fusion.py`'s fusion runs beside it in the same process, an epoch every half second, and publishes the `RDR_FUSED_TRACKS` block. The status and both blocks are broadcast (`BCAST_RDR_TGT_STATUS` 31/24 for the MFD and the radar display, `BCAST_RDR_TGT_TRACKS` 31/20-21 for the MFD, `BCAST_RDR_FUSED_TRACKS` 31/22-23 for the EICAS and the TSD), as is the FMS's energy maneuverability for the TSD (`BCAST_FMS_ENERGY` 31/28); the displays take them up as each is wired. The terrain block drops to 25 Hz with a profile every fifth of a second, which is what paid for the new blocks within the bus budget. Enums `target_class`, `iff_identity`, `threat`, `lock_status`.

**v1.11** puts the TSD (RT 16) on the bus (`fmofp_display tsd`) and gives the targeting radar its displays. The TSD adds itself to the listeners of the FMS's flight data, status and attitude status (`BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS`, `BCAST_FMS_ATTITUDE_STATUS`), hears the energy maneuverability and the fused tracks that v1.10 broadcast for it (`BCAST_FMS_ENERGY`, `BCAST_RDR_FUSED_TRACKS`), and commands nothing. The MFD's radar page draws the targeting radar's tracks from `BCAST_RDR_TGT_STATUS` and `BCAST_RDR_TGT_TRACKS`, the radar display commands that radar through `DSP_RDR_TGT_CMD` (relayed since v1.5, now with a sender) and shows its status, and the EICAS counts the hostile fused tracks. Nothing new goes on the bus: three listener lists grow by one terminal.

**v1.12** adds the AEWC radar to the radar computer (RT 9, `fmofp_radar`). It reads the same `RDR_TRAFFIC` block as the targeting radar, out to its 400 km, and keeps `aewc_radar.py`'s track table, radar equation, classification rule and six sectors, with `aewc_processor.py`'s sector priority manager and electronic protection: `RDR_AEWC_STATUS` grows to three words (track and stealth counts, the jammed sectors, the priority sector), the TBD sector block becomes `RDR_AEWC_TRACKS` (the tracks and the six sectors, at 10 Hz), and both are broadcast for the MFD's AEWC page and the radar display's status line (`BCAST_RDR_AEWC_STATUS` 31/11, `BCAST_RDR_AEWC_TRACKS` 31/16-17). Its reports join the fusion (weight 0.7), so the fused tracks now say `aewc_present` and merge a near aircraft seen by both radars. The radar display commands it through `DSP_RDR_AEWC_CMD`. Enums `jam_type`, `ecm_mitigation`.

**v1.13** adds the synthetic aperture radar to the radar computer, the last of the five (RT 9, `fmofp_radar`), and the ground to the plant: `sar_radar.py` drew its images from nothing (random bars, a disc, swaths, noise), so the plant now holds a training range laid out along the northbound track from home (`fmofp_plant.ground`: the road beside the track, a town with its buildings, a river, forests, a lake with a ship under way, vehicles on the roads), published as the `PLANT_GROUND` block (relayed as `RDR_SAR_SCENE`), and the radar makes its picture of whatever lies in its strip. The radar computer becomes a listener of `BCAST_FMS_FLIGHT_DATA`: the SAR places its pictures by the position and heading in it. The TBD `RDR_SAR_IMAGE` block gets its layout: a 64 x 64 picture (the Python's 1024 x 1024 image reduced sixteen-fold on each side) with its geometry and `sar_processor.ChangeDetector`'s change regions, an image every 150 frames; `RDR_SAR_STATUS` says whether the radar has a scene and a fix and which image it last published. Both are broadcast for the MFD's SAR page and the radar display's status line (`BCAST_RDR_SAR_STATUS` 31/15, `BCAST_RDR_SAR_IMAGE` 31/13-14), and the radar display commands the radar through `DSP_RDR_SAR_CMD`, with the resolution. Enums `ground_feature`, `change_type`. The eleven-terminal aircraft reads 6.7 ms of the 10 ms budget (the image segments at 25 Hz, the scene at 10 Hz; 6.37 ms before).

**v1.14** puts the SMS (RT 17) on the bus (`fmofp_display sms`). It adds itself to the listeners of `BCAST_FMS_STATUS`, whose flight mode and tactical systems its master arm, ECM cell and signature strip follow, and commands nothing; nothing new goes on the bus. Its weapon stations stay the display's own table, as in the Python, which modelled no armament system (section 2).

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
| Display data goes out once, by broadcast | v1.7 | A display's copy of a message costs the bus a transfer, and the MFD's copies had pushed the schedule past its budget (v1.6). From v1.7 the radars' status, scan summary, guidance and product blocks, the flight computer's alerts and exceedances and the FMS's flight data, status and attitude status are relayed once each to address 31, and every display that wants one accepts that subaddress for broadcast (`RemoteTerminal::accept_broadcast`, `fmofp_bus.remote_terminal.RemoteTerminal.accept_broadcast`); a terminal that has not accepted a subaddress ignores a broadcast on it, so the radar's own command subaddresses are safe from the displays' data on the same numbers. Nobody answers a broadcast, so there is no retry: a display that misses a frame of a message waits for the next, and a block segment it misses costs it that version of the block, which the version-and-CRC rule already covers. Flight-critical data keeps its unicast relay (`FCS_TERRAIN`). The broadcast map is program-wide: 1-10 the radar and flight-control band, 25-27 the flight-management band, clear of every display's own subaddress (11-17); the tool refuses a listener that has a receive message of its own on a broadcast subaddress. With eight terminals the busiest frame is 7.1 ms (35%) (8.1 ms with the v1.6 copies) |
| BIT merges into avionics; FM fitness and SW configuration fold into another terminal | Alee | All three run in the avionics process at RT 1. FM fitness and SW configuration went there because both are health and maintenance functions; Alee can move them |
| Bus controller is its own process and the master clock | Alee | Every minor frame starts with a broadcast sync (mode code 17); terminals run their frame on receipt |
| One process per display | Alee | Each display needs its own RT, because one address is one terminal. RT 11 goes to the PFD; MFD, EICAS, radar display, TSD, SMS and HUD get RT 13–18 |
| Keep the current RT addresses | Alee | RT 0–10 and 12 unchanged. MIL-STD-1553B doesn't assign addresses to types of equipment (each program's ICD does), so there is no more conventional set to move to |
| Big-endian on the socket | Alee | Every word, header included |
| UDP on 127.0.0.1, port base 51000 | Alee | Section 7 |
| One machine-readable file generates both sides | Alee | XML, because Python reads it with the standard library and it matches the existing config files |
| Freeze the Python code until the conversion is finished | Alee | The port is measured against upstream `main` at `0ab9236` (2026-10-08): that is the Python each partition was translated from. Upstream kept moving, so from 2026-10-25 (Kevin) its commits are merged into `port/cpp` whenever they merge cleanly and change none of the mirrored logic (the ICD sources, the ported modules, the modules the oracle tests import, the display classes the display processes embed); a merge that would is not made, and is reported. The oracle tests, which compare the C++ with the Python in the tree, arbitrate every merge |
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
| The engine is simulation; thrust, autothrottle and fuel management are flight software | v1.8 | `engineControlUnit.py`'s single engine (N1, N2, EGT and fuel flow from the thrust, oil pressure, oil temperature and vibration from sine waves of the time, each with Gaussian noise) runs in the plant, twice, as `PLANT_ENGINE`, with simulated seconds for its time. The engine computer (RT 3) carries `thrustManagement.py` and `fuelControl.py` with `fuelMonitor.py`, and the assessment `mfd.draw_engine_page` made of the ECU's numbers (red at N1 or N2 100 %, EGT above 800 C, oil pressure below 40 psi, oil temperature above 130 C; amber for vibration above 0.8) travels as bits in `ENG_ENGINE_STATUS` |
| The FMS thrust lever drives the engines | v1.8 | In the Python, `fmsControl.set_thrust`'s thrust, `ThrustManagementSystem`'s per-engine values and the ECU's thrust were three numbers that nothing connected; the ECU's random-walked because nothing commanded it. Now `FMS_CONTROL_STATUS.throttle_position` (relayed as `ENG_FMS_CONTROL`) commands both engines through `set_all_thrust` whenever it moves, `ENG_CMD SET_THRUST` commands one or both, and the autothrottle owns them while engaged (after a disengage the thrust stays where it left it until the lever moves). `ENG_THRUST_CMD.commands_valid` is 0 until something has commanded the thrust, and the plant's engines keep their own setting (the ECU's starting 70 %) until then. The thrust is the command at the engines' next 2 Hz step: no spool-up (section 12) |
| The fuel burns from the measured fuel flow | v1.8 | `FuelSystem.update` drew each engine's `fuel_consumption_rate` from every tank on its lines in full, and that rate was `0.1 x thrust` from a `set_engine_thrust` nothing called, so the Python's tanks never moved. The engine computer burns each engine's measured fuel flow split by its fuel lines' weights (main 100 : Center 50), runs `perform_fuel_transfer`'s policy (a main below 5000 kg receives 1000 kg while the Center holds more than 5000) and `FuelMonitor.poll`'s rules (low at 20 %, critical at 10 %, an unexplained loss when a tank drops more than the burn explains by over 2 % of its capacity); transfers count toward the explained drop and not toward the consumption rate. `transfer_fuel` moves no more than the destination can take, where the Python's `refill` lost the overflow |
| The autothrottle's clock | v1.8 | `update_autothrottle` ran in a 2 Hz thread with the wall clock's `dt`; the engine computer runs it every 25 frames with `dt` 0.5 s on the airspeed of `BCAST_FMS_FLIGHT_DATA`, and skips a step whose airspeed is stale (three periods, 0.3 s). The PI gains, the integral limit and the asymmetry limit are the Python's, checked against it by `test_engine_oracle.py` |
| Fuel efficiency | v1.8 | `FCS_PERF_STATUS.fuel_efficiency` is `performaneMonitoring._compute_fuel_efficiency` (airspeed / (fuel flow / 60), 0 for no flow) on `ENG_ENGINE_STATUS.total_fuel_flow` heard by broadcast; NO_COMPUTED_DATA while that is absent or stale (three periods of its 5 Hz, 0.6 s) |
| Subsystems hear broadcasts too | v1.8 | A broadcast's `listeners` may name a subsystem as well as a display: the engine computer hears `BCAST_FMS_FLIGHT_DATA` (31/25) and the flight control computer `BCAST_ENG_ENGINE_STATUS` (31/18), so neither needs a relayed copy of its own. Flight-critical inputs stay unicast relays (`FCS_TERRAIN`); these two feed monitoring and the autothrottle's reference, which a missed frame does not hurt |
| The EICAS draws what the bus vouches for | v1.9 | `eicas._poll_data` polls the FMS for its thrust (the flight profile's engine power), the ECU for the engine readings, the GCAS and the performance monitor for alerts, the built-in test controller, the defensive, sensor and fusion services and the weather radar's processors. The feed answers each from the broadcasts. The fuel quantity, the left-right balance and the oil readings, which `eicas.py` left UNKNOWN for want of a feed, go in through its own `set_engine_readings()` and are alarm-checked (the 1000 kg transfer of the demo flight shows as FUEL IMBALANCE). One quirk of `eicas.py` is kept: it computes its alert list before it copies the ECU's readings in, so the EGT alert grades the thrust model's EGT (350 + 4.5 x the profile's power) and reads ENG EGT CAUTION at COMBAT power whatever the engine measures, while the panel shows the measurement. Hydraulics beyond the g-load model, electrical and ECS stay UNKNOWN, the display's own honest state, until their terminals exist; BITS, defensive, sensors and fusion read as they did with those services not running |
| The flight profile's engine power is always given | v1.9 | `eicas._compute_alerts` compares the thrust unconditionally (a None would raise and end the poll), so `FlightDataFeed.get_tactical_status()` reports NORMAL's 0.7 until the FMS has said which mode it is in: `fmsControl`'s own state from construction. The PFD's limits in the same dictionary keep `pfd.py`'s defaults |
| Wind shear severity in the scan summary | v1.9 | `RDR_WX_SCAN.max_shear` (word 1 bits 14-13) is the strongest event's severity, beside `max_turbulence` and `microburst`, which the summary already carried for a consumer that does not reassemble `RDR_WX_PRODUCTS`; the EICAS's wind shear alert reads it |
| The air traffic is test equipment; the radars measure it | v1.10 | `targeting_radar._generate_target` and `aewc_radar._generate_target` drew their targets around the aircraft in a frame that moved with it and never knew it moved, and `_update_targets` flew them on every track request with the wall clock's `dt`. The plant now owns the traffic: the same draws (five aircraft within 100 km for the targeting radar's population, seven within 400 km for the AEWC's, three in ten of those stealth; the Python AEWC kept up to ten, seven fit the block beside the five), the same kinematics at the plant's 10 Hz in a frame fixed to the plant's home, an aircraft replaced when it leaves its generator's range as the Python deleted and redrew it. It publishes what the antennas would measure, slant range, true azimuth and elevation, the velocity relative to the aircraft (its own motion subtracted, which the Python never did) and the radar cross-section, as `PLANT_TRAFFIC` five times a second. Nothing in the block says what an aircraft is; the radars classify from what they measure |
| Tracks are associated by position | v1.10 | The Python's `current_targets` were keyed by the generator's own ids, so a track was never lost or confused. The radar continues the held track whose predicted position (last position plus velocity times the time since) is nearest the detection within 1000 m, starts a new track otherwise, and drops a track unseen for more than 15 frames (traffic arrives every 10). Track ids are the radar's own, from 1, kept while the track is held |
| The signal model is the Python's, with repeatable draws | v1.10 | SNR 30 - 20 log10(range / 1000) dB; confidence and lock quality min(1, snr / 30), floored at 0 (the Python's went negative beyond 31.6 km, the words cannot); LOCKED above 0.5, else ACQUIRING; the 5 % jamming draw per update while locked. `SignatureAnalyzer`'s Swerling scintillation (`random.expovariate(1.0)`) and the jamming draw come from the radar's own repeatable generator (the weather radar's texture draw) keyed by track id and update count, so a seeded flight is the same flight twice and `test_targeting_oracle.py` can hand the Python the same value |
| One command for mode, filter and lock | v1.10 | The Python had `set_mode`, a track filter in each track request (`track_type`) and a separate `LOCK_REQUEST` message, refused outside TARGET_TRACK. `RDR_TGT_MODE_CMD` carries all three: `lock_track_id` 0 releases the lock (new), a track the radar does not hold leaves the lock as it is (the Python logged and ignored it), and a lock request outside TARGET_TRACK is ignored as the Python's was. A command whose state is not NORMAL clears the tracks and the lock as `set_mode(STANDBY)` did. The filter selects what `RDR_TGT_TRACKS` reports, by the threat rule applied to each track; the tracking itself is unfiltered, and `RDR_TGT_STATUS.track_count` counts every held track |
| The fusion lives in the radar computer | v1.10 | `RadarDataFusion` was a singleton thread polling the targeting and AEWC radars' tables every half second. Both are in RT 9, so it runs there, an epoch every 25 frames over the radars' current tracks: nearest-centroid clustering within 500 m, heaviest sensor first (targeting 0.8, AEWC 0.7), weight-averaged position and velocity, confidence the probability union of the weights, the heaviest report's class and identity, `_assess_threat`'s rule on the average RCS and the first report's speed, ids carried from the nearest unused previous track within 500 m. `test_targeting_oracle.py` checks an epoch against the Python (its stable sort by weight; its `max()` keeping the first of equals). Its 30 s track ageing is not carried: the table is rebuilt from the sensors' tracks every epoch, as the Python's was, so nothing was ever left to age. The AEWC's reports join when that radar is ported; `aewc_present` says whether it contributed |
| Identity is UNKNOWN until an IFF exists | v1.10 | The Python radars gave every target the identity 'UNKNOWN', so the threat rule always fell through to its heuristic (HOSTILE when the average RCS is under 3 m2 and the speed over 200 m/s). `iff_identity` is carried so an interrogator can fill it in later |
| The terrain block at 25 Hz | v1.10 | With the traffic, the tracks and the fused tracks, the ten-terminal aircraft's busiest frame reached 10.09 ms of the 10 ms budget. `PLANT_TERRAIN` was polled every frame for a profile published ten times a second (four segments in four frames); it is now polled every other frame for a profile published five times a second (four segments in eight frames of the ten), during which the aircraft covers half a sample's spacing. The radar's staleness window for the profile is three periods (30 frames). The busiest frame is 8.72 ms with ten terminals |
| The TSD draws what the bus vouches for | v1.11 | `tsd._poll_data` reads the FMS flight data (heading, altitude, airspeed, g, angle of attack, energy state, mode, position), `fmsControl.get_tactical_status()` (tactical systems, envelope warnings, energy metrics, the profile's engine power), `RadarDataFusion.get_fused_tracks()` (drawn through `FusedTrack.to_tsd_dict()`), the mission service's route and order of battle and the defensive service's radar warning contacts. The feeds answer the first three from the broadcasts: the flight data feed gains the position, the tactical systems (the FMS status's countermeasures, targeting, weapons and stealth bits, by their names) and the energy metrics of `BCAST_FMS_ENERGY`, each left out until the bus has vouched for it, so the TSD keeps its own defaults as it did when fmsControl answered without them; the fused tracks are the block's records with the Python's bearing (atan2(east, north)), range (three-dimensional, in nautical miles), type, hostile flag, confidence, sources, id and identity, and no velocity, as `to_tsd_dict()` carried none (the TSD's vector code never ran on a fused track). The mission and defensive services answer as they did when not running: no route, no order of battle, no contacts; and the TSD says OWN POS UNKN and withholds the overlay until the position is valid, as `tsd.py` does |
| The targeting picture is the MFD's radar page | v1.11 | In the Python the targeting radar's picture was the MFD's `TargetingRadarDisplay` (the radar display window showed the weather radar alone); the port keeps it there and gives the radar display the controls, as for the other radars: `DSP_RDR_TGT_CMD` carries the mode, the track filter and the track to lock, and the window's status line shows the radar's status. `RadarFeed.targeting_data()` hands the MFD the tracks block's records as the display reads them: positions in nautical miles east, north and up of the aircraft (the display's `world_to_screen` takes nautical miles against its range scale, set to the radar's 100 km), the velocity relative to the aircraft, the id, and the identity and classification in the display's own vocabulary. The Python radar's class names (FIGHTER, BOMBER, ...) never matched the display's symbol table (FIXED_WING, ROTARY_WING, SURFACE), so every target would have drawn as a '?'; the feed maps the aircraft classes to FIXED_WING and HELICOPTER to ROTARY_WING, leaves MISSILE and UNKNOWN to the '?', and an UNKNOWN identity to the display's yellow |
| The EICAS counts the fused hostiles | v1.11 | `eicas._poll_data` counts `get_radar_data_fusion().get_threat_tracks()` for its FUSION alerts (one or more hostile tracks a caution, three a warning); the feed answers from the fused tracks block the EICAS now accepts. Its TRACKS row reads the sensor service's `fused_track_count`, a different source, which waits for the sensor management terminal |
| The AEWC radar measures the plant's far traffic | v1.12 | `aewc_radar._generate_target` invented up to ten aircraft within 400 km (three in ten stealth) and `_update_targets` flew them; the plant has flown that population since v1.10 (seven aircraft, the block's room beside the targeting radar's five) and the AEWC now tracks what `RDR_TRAFFIC` says of every aircraft within 400 km, the targeting radar's near ones included, by the same nearest-prediction association (1000 m, dropped after 15 unseen frames). It classifies by `_generate_target`'s own rule from what it measures: a cross-section under 0.1 m2 is STEALTH (the generator's stealth band ends there and its conventional band starts at 1 m2), a speed over 400 m/s FIGHTER, more than 30 km above or below HIGH_ALT, else UNKNOWN |
| The radar equation as the Python read it | v1.12 | `_calculate_snr` computes the received power in watts (100 kW, 40 dB gain squared, a 3 cm wavelength, the cross-section, over (4 pi)^3 R^4), takes 10 log10 of it and subtracts a -110 dBm floor, so a dBW number is read as dBm and the SNR is 30 dB low; it is then clipped at 0. Kept as written: beyond about 40 km every track has SNR 0 and confidence 0, a stealth aircraft is detected with probability 0.2 x snr / 30 and so almost never beyond 10 km. The detection draw comes from the radar's repeatable generator; a missed detection keeps the track where the measurement put it (the Python's `continue` came after the position update) and leaves it out of the sector counts |
| Sectors, priorities and electronic protection as written | v1.12 | Six 60-degree sectors by `atan2(north, east)` (counter-clockwise from east, the Python's convention) within -15 to 45 degrees of elevation; the scan progress advances a tenth a second; `SectorPriorityManager.update` scores each sector by its tracks, hostile tracks (none: nothing marks a track hostile for it), the seconds since its last scan (`mark_scanned` was never called, so the age grows from the radar's start) and 2 points when jammed, and gives it a dwell share of at least 5 %; `ElectronicProtection.assess_sector` grades the receiver noise exceedance over the thermal floor, which in the Python was its own `gauss(0, 3)` draw, so a sector reads jammed about 2 % of the time and clears at the next pass. The block carries all of it per sector; `RDR_AEWC_STATUS` carries the jammed mask and the priority sector |
| The AEWC picture is the MFD's radar page, with one display fix | v1.12 | `AEWCRadarDisplay` draws stealth targets as purple diamonds and hands conventional ones to `TargetingRadarDisplay._draw_target`, but called it with the rectangle where it takes the centre and the radius: every conventional target raised TypeError and the base class drew its error message instead of the picture, so the page could never show one. That call is corrected in `aewc_radar_display.py`, the one change to a display class in the port; its missing `_draw_ground_mapping_mode` is left alone, and the feed shows the search picture for GROUND_MAPPING. The feed hands the MFD the block's tracks as `RadarFeed.aewc_data()` (positions in nautical miles against the radar's 400 km, the display's symbol vocabulary, `is_stealth`); the radar display commands the radar and shows its status |
| Relays cost one RT-to-RT transfer | v1.12 | With the AEWC block the eleven-terminal aircraft stood at 9.7 ms of the 10 ms budget under an estimate that charged every relay as a poll plus a write per consumer, which is what the emulated link does (the controller reads the words and sends them on) but not what a real controller does: it commands an RT-to-RT transfer, two command words and two status words around the data crossing once, and hears the data itself. The estimate now charges that (`Schedule`, `fmofp_bc --schedule`), a broadcast consumer one transfer as before; the eleven terminals read 6.37 ms. The link is unchanged |
| The AEWC block at 10 Hz | v1.12 | Its six segments (twelve tracks of twelve words, six sectors of four) take 30 frames at 10 Hz, so it is republished no more often than every 35 frames; the 10 Hz rate group has five phases to spread over, where the 25 Hz group's two were nearly full. A wide-area picture every 0.7 s is what the Python's track requests gave the displays |
| The SAR images the plant's ground | v1.13 | `sar_radar._generate_imagery_data` made a 1024 x 1024 picture of random bars (STRIPMAP), a disc (SPOTLIGHT) or swaths (SCANSAR) with gaussian noise, on request, with nothing behind it. The port keeps the line the weather, the terrain and the traffic drew: the world is the plant's. `PLANT_GROUND` carries a fixed training range as point scatterers, lines and discs with the mean backscatter each would return (buildings 220-240, a town 150, a forest 100-110, open ground 60, a road 20, a lake 6), and the radar renders the strip beside its track from it: areas paint the ground, lines cover the fraction of a pixel their width crosses, points the fraction of a pixel their square overlaps (the native image block-averaged), then four-look speckle. What a feature *is* never reaches the radar, only its backscatter |
| The picture on the bus is the Python's image, sixteen-fold reduced | v1.13 | The Python's 1024 x 1024 image at `resolution` metres per pixel would be 524288 words: 109 minutes of bus time at a segment every frame, where a real SAR has a dedicated imagery link. The bus carries 64 x 64, so a pixel is 16 native pixels (64 in SCANSAR, which trades resolution for swath), the commanded resolution sets the patch (a 1 km strip at 1 m, 300 m at 0.3 m) and a vehicle shows only at a fine one. The 2075-word block takes 67 segments, 134 frames at 25 Hz, so an image is formed every 150 frames; the three seconds read as the integration time a SAR needs |
| The strip lies 2 km to the right; a spotlight stares at the patch abeam | v1.13 | A side-looking radar images beside its track; the near edge sits 2 km to the right of the aircraft and the picture's top is along the heading. `RDR_SAR_MODE_CMD` has no target point (nor had the Python), so SPOTLIGHT is designated by flying the target abeam and commanding the mode: the patch is frozen where the aircraft was, with its heading, until the mode or the resolution changes. A designated point is backlog (section 12). INTERFEROMETRIC and DOPPLER_BEAM, which the Python drew as noise, image the strip; the MFD's display draws standby for them, as it always did |
| The change detector as written, reset on a mode change | v1.13 | `sar_processor.ChangeDetector` runs on the picture pair as in the Python (log-ratio, local mean plus three local standard deviations over an 11 x 11 window, opening and closing, 4-connected regions of ten pixels or more, up to 32), and the port matches it event for event (`test_sar_oracle.py`). Two things about it are the Python's and are kept: the CFAR window includes the pixel's own neighbourhood, so a compact change of more than about a tenth of the window raises the threshold above itself and is never reported (a moving ship is not), and the running-sum box filter leaves a rounding residue a few ulps below zero where its window has passed a change, so a log-ratio of exactly zero beside it is reported as CHANGE. The one departure: `reset_reference` is called on a mode or resolution change, as its docstring asked and `sar_radar.py` never did, since a new geometry is not a change on the ground |
| The radar computer listens to the flight data broadcast | v1.13 | The SAR needs a position and a heading. The FMS flight data already goes out once to address 31 (31/25) for the displays and the engine computer; RT 9 adds itself to the listeners instead of a 21-word relay of its own, as the engine computer did in v1.8 |
| The SAR broadcasts in the displays' own band | v1.13 | `BCAST_RDR_SAR_STATUS` 31/15 (the TSD's number: only the TSD is barred, and it does not listen), `BCAST_RDR_SAR_IMAGE` 31/13-14 (the EICAS's and the radar display's numbers; the MFD, the one listener, has no receive of its own there), under the per-listener rule of section 8 |
| The SMS draws its own loadout; its master arm follows the FMS | v1.14 | `sms._poll_data` reads `fmsControl.get_tactical_status()` five times a second for the mode and the tactical systems, and derives the master arm from the weapons state (ARMED is ARMED, READY is TRAIN, else SAFE), the ECM cell from the countermeasures state (ACTIVE; STANDBY for READY and PASSIVE; else INACTIVE) and the signature from the stealth bit (0.001 m2 and 5 % IR with stealth ON, else a slow sine of wall-clock time standing in for a sensor suite). The feed answers it from `BCAST_FMS_STATUS`, by the names the Python used, and omits the tactical systems until the bus has vouched for them, which the display's own `.get('weapons', 'SAFE')` reads as a safe master arm. The station table (two internal bays, two wingtip decoy launchers, a centreline pod), the release and delivery modes and the selected station are `_build_default_loadout()` and class constants, because no stores management system exists in the Python to report them; they stay the display's, and a stores management computer that would own them is backlog (section 12). The defensive service's chaff and flare counts, which a stores page would show, wait for RT 22. One look of the display is the Python's too and stays: its station table is tinted amber, because `_draw_detail` outlines its panel with `drawRect` while the brush of the last station icon drawn (the amber ECM pod, alpha 100) is still set on the painter; the table reads, so the display class is left as it is (a one-line display ticket for later) |

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

**Added in v1.7:**

| Field | Now | Taken from |
|---|---|---|
| Broadcast messages | `BCAST_RDR_WX_STATUS`, `BCAST_RDR_WX_SCAN`, `BCAST_RDR_TFR_STATUS`, `BCAST_RDR_TFR_GUIDANCE`, `BCAST_FCS_GCAS_ALERTS`, `BCAST_FCS_PERF_STATUS`, `BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS`, `BCAST_FMS_ATTITUDE_STATUS` and the blocks `BCAST_RDR_WX_PRODUCTS`, `BCAST_RDR_TFR_ELEVATION`, each with its `listeners` | The v1.5 and v1.6 per-display copies (`DSP_RDR_*`, `PFD_*`, `MFD_*`), which they replace |
| The broadcast map | Subaddresses 1-10 (radar pictures, flight control), 18-19 (engine), 20-24 (tracks), 25-28 (flight management), and 11, 13-17 for the AEWC's and the SAR's data under the per-listener rule, at address 31; a terminal's `accept_broadcast` per subaddress | MIL-STD-1553B broadcast (address 31, no status word), with the acceptance rule standing in for a real terminal's legal-command table |

**Added in v1.8:**

| Field | Now | Taken from |
|---|---|---|
| Engine measurements | `PLANT_ENGINE` (plant, RT 27 SA 6, 10 Hz), relayed as `ENG_ENGINE_IN` | `EngineControlUnit._adjust` and `_snapshot`: thrust_pct, n1_pct, n2_pct, egt_c, ff_kgh, oil_psi, oil_temp_c, vibration, compressor_eff, for each of the two engines |
| Thrust command | `ENG_THRUST_CMD` (RT 3 SA 14, 10 Hz), relayed as `PLANT_THRUST_CMD` | `ThrustManagementSystem.get_all_thrust()`, `autothrottle_engaged`, `check_asymmetry()`; the FMS's afterburner bit passed on |
| Engine computer inputs | `ENG_FMS_CONTROL` (a relay of `FMS_CONTROL_STATUS`), `BCAST_FMS_FLIGHT_DATA` with RT 3 among its listeners | `_update_loop`'s `fms.get_flight_data()['velocity']['airspeed']`; the FMS thrust lever, which nothing read before |
| Engine computer commands | `ENG_MODE_CMD`; `ENG_CMD` with SET_THRUST, AUTOTHROTTLE_ENGAGE, AUTOTHROTTLE_DISENGAGE, FUEL_TRANSFER | `set_thrust`, `set_all_thrust`, `engage_autothrottle`, `disengage_autothrottle`, `FuelManagementSystem.transfer_fuel`: entry points nothing in the Python called |
| Engine computer outputs | `ENG_STATUS` (1 Hz); `ENG_ENGINE_STATUS` (5 Hz; broadcast as `BCAST_ENG_ENGINE_STATUS`, 31/18, to the MFD and the FCS); `ENG_FUEL_STATUS` (1 Hz; broadcast as `BCAST_ENG_FUEL_STATUS`, 31/19, to the MFD) | `get_status()` of the thrust and fuel management systems, `FuelMonitor.get_alerts()` and `get_consumption_rates()`, the MFD engine page's thresholds |
| Enums | `engine_command`, `engine_select`, `fuel_tank` | The two engines and three tanks of `FuelManagementSystem.setup_aircraft` |

**Added in v1.9:**

| Field | Now | Taken from |
|---|---|---|
| EICAS inputs | RT 14 among the listeners of `BCAST_ENG_ENGINE_STATUS`, `BCAST_ENG_FUEL_STATUS`, `BCAST_FCS_GCAS_ALERTS`, `BCAST_FCS_PERF_STATUS`, `BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS` and `BCAST_RDR_WX_SCAN` | `eicas._poll_data()`: `get_flight_data()`, `get_tactical_status()['profile_limits']['engine_power']`, `get_engine_control_unit().get_data()`, `get_gcas().get_alerts()`, `get_performance_monitor().get_exceedances()`, the weather radar's `windshear_proc` and `turbulence_proc`; `set_engine_readings()` for the oil readings, the fuel quantity and the balance |
| `RDR_WX_SCAN.max_shear` | Two bits of the spare in word 1 | `WindShearProcessor`'s strongest event (`severity_level()`), which `eicas._compute_weather_alerts` grades |

**Added in v1.10:**

| Field | Now | Taken from |
|---|---|---|
| Traffic | `PLANT_TRAFFIC` block (RT 27, descriptor 8 / segment 9, 25 Hz, a publication every 0.2 s), relayed as `RDR_TRAFFIC` (RT 9, 8/9): sequence, up to 12 targets of plant id, true azimuth (360/65536 deg), slant range (1/64 km), elevation (180/32768 deg), relative velocity east / north / up (1/8 m/s), RCS (1/100 m2) | `targeting_radar._generate_target` / `aewc_radar._generate_target` (ranges 1-100 km / 5-400 km, elevations within 30 / 45 degrees, speeds 100-300 / 150-500 m/s, accelerations up to 30 / 50 m/s2, RCS 1-10 / 1-100 m2, the AEWC's stealth 0.01-0.1 m2 three in ten), `_update_targets` |
| Targeting status | `RDR_TGT_STATUS` (RT 9 SA 4, 4 words, 5 Hz): state, mode, last command, rejected, mode transition, track count (saturating at 31), lock status, jamming, track filter, healthy, locked track id, lock quality (1/1000) | `targeting_radar.get_status()`, `_handle_track_request`'s lock data, `TargetingRadarStatusData` |
| Targeting command | `RDR_TGT_MODE_CMD` (RT 9 SA 4, 3 words): state, mode, cmd_seq, track filter, lock track id | `set_mode`, `_handle_track_request`'s `track_type`, `_handle_lock_request` |
| Tracks | `RDR_TGT_TRACKS` block (RT 9, 10/11, 25 Hz, republished at most every 15 frames): update sequence, mode, lock status, jamming, locked track id, lock quality; per track (12 words, up to 12): id, x / y / z (double words, 1/16 m, east / north / up of the aircraft), velocity (1/8 m/s), classification, identity, threat, stealth, locked, confidence (1/100), stealth probability (1/100) | `TargetingRadarTrackData` (position, velocity, classification, confidence, is_stealth, stealth_probability); `target_processor.SignatureAnalyzer` (`_RCS_REFERENCE`, the speed and altitude overrides, confidence 0.2-0.95 from the log-RCS distance); `StealthDetector` (prior 0.05, likelihood ratios 8 / 2 / 0.2 by RCS band, at least 8 pulses integrated, decision 0.6) |
| Fused tracks | `RDR_FUSED_TRACKS` block (RT 9, 24/25, 25 Hz, a version per epoch): epoch, hostile count, targeting present, AEWC present; per track: id, x / y / z, velocity, classification, identity, threat, from targeting / from AEWC, confidence | `RadarDataFusion` (`GATE_M` 500, `_W_TARGETING` 0.8, `_W_AEWC` 0.7, `UPDATE_HZ` 2), `FusedTrack.to_tsd_dict()`, `get_threat_tracks()` |
| Broadcasts | `BCAST_RDR_TGT_STATUS` 31/24 (listeners 13, 15), `BCAST_RDR_TGT_TRACKS` 31/20-21 (13), `BCAST_RDR_FUSED_TRACKS` 31/22-23 (14, 16), `BCAST_FMS_ENERGY` 31/28 (16) | The MFD's targeting page (`targeting_radar_display`), `eicas.get_threat_tracks()`, `tsd._poll_data`'s fused tracks and energy metrics |
| Enums | `target_class` (UNKNOWN, FIGHTER, BOMBER, HIGH_ALT, MISSILE, HELICOPTER, DRONE, STEALTH), `iff_identity` (UNKNOWN, FRIENDLY, HOSTILE, NEUTRAL), `threat` (UNKNOWN, HOSTILE, FRIENDLY), `lock_status` (NONE, ACQUIRING, LOCKED) | `_RCS_REFERENCE`'s keys and `aewc_radar`'s 'STEALTH'; `radar_data_fusion`'s identities and threat classes; `_handle_track_request`'s lock status strings |
| Terrain cadence | `PLANT_TERRAIN` and `RDR_TFR_TERRAIN` at 25 Hz; the plant publishes a profile every 0.2 s | The bus budget (section 2) |

**Added in v1.11:**

| Field | Now | Taken from |
|---|---|---|
| TSD inputs | RT 16 among the listeners of `BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS` and `BCAST_FMS_ATTITUDE_STATUS` (it was already the listener of `BCAST_FMS_ENERGY` and one of `BCAST_RDR_FUSED_TRACKS`) | `tsd._poll_data()`: `get_flight_data()`, `get_tactical_status()`, `get_fused_tracks()`, `get_nav_data()`, `get_oob()`, `get_rwr_contacts()` |
| Targeting commands | `DSP_RDR_TGT_CMD` (RT 15 SA 23, the layout of `RDR_TGT_MODE_CMD`, relayed since v1.5), now sent by the radar display window's targeting buttons | `targeting_radar.set_mode`, `_handle_lock_request` |

**Added in v1.12:**

| Field | Now | Taken from |
|---|---|---|
| AEWC status | `RDR_AEWC_STATUS` (RT 9 SA 5, 3 words, 5 Hz): state, mode, last command, rejected, mode transition, track count (saturating at 31), stealth tracks (at 15), healthy, jammed sectors (bit per sector), priority sector | `aewc_radar.get_status()` (`active_tracks`, the sectors), `ElectronicProtection.get_jammed_sectors()`, `SectorPriorityManager.update()`'s first sector |
| AEWC tracks and sectors | `RDR_AEWC_TRACKS` block (RT 9, 12/13, 10 Hz, republished at most every 35 frames): update sequence, mode, the commanded sector's azimuths; per track (12 words, up to 12): id, x / y / z, velocity, classification, identity, threat, stealth, detected, sector, confidence (1/100), SNR (1/2 dB); per sector (4 words, six): first azimuth, detected tracks, jammed, scan progress (1/100), dwell fraction (1/100), priority score (1/10), jam type, mitigation, severity (1/100) | `AEWCRadarTrackData` (position, velocity, classification, `track_confidence` = snr / 30), `AEWCRadarSectorScan.sector_bounds` (azimuths, `scan_progress`, `active_tracks`), `SectorStatus` (`priority_score`, `dwell_fraction`), `JammerContact` (`jam_type`, `severity`, `mitigation_active`) |
| Broadcasts | `BCAST_RDR_AEWC_STATUS` 31/11 (listeners 13, 15), `BCAST_RDR_AEWC_TRACKS` 31/16-17 (13); subaddresses in the displays' own band, which only bars the display that owns each number (the PFD, the SMS, the HUD) from listening | The MFD's AEWC page (`aewc_radar_display`), the radar display's status line |
| Enums | `jam_type` (NONE, DECEPTIVE, SWEEP, SPOT, CONTINUOUS), `ecm_mitigation` (NONE, FREQ_AGILITY, BEAM_NOTCH, POWER, SKIP) | `ElectronicProtection.assess_sector` and `_select_mitigation`'s strings |

**Added in v1.13:**

| Field | Now | Taken from |
|---|---|---|
| SAR status | `RDR_SAR_STATUS` (RT 9 SA 3, 2 words, 5 Hz) gains scene_valid, nav_valid and image_seq (the last image's number, 0 before the first); the mode transition ends with the first image in the new mode | `sar_radar.get_status()` |
| Ground scene | `PLANT_GROUND` block (RT 27, 10/11, 10 Hz, a publication a second), relayed as `RDR_SAR_SCENE` (RT 9, 26/27): sequence; points (6 words, up to 10): kind, brightness, latitude, longitude, size; lines (10 words, up to 3): kind, brightness, both ends, width; areas (6 words, up to 4): kind, brightness, centre, radius | `fmofp_plant.ground`'s training range (new; the Python had no ground) |
| SAR image | `RDR_SAR_IMAGE` block (RT 9, 16/17, 25 Hz, an image every 150 frames): image_seq, mode, significant changes, resolution, pixel_size, the centre's latitude and longitude, heading, centre_along, centre_cross; 2048 pixel words (64 rows of 64, two pixels a word); change regions (3 words, up to 5): row, col, area, type, significant, confidence, log_ratio | `sar_radar._generate_imagery_data` (1024 x 1024 at `resolution`), `sar_processor.ChangeEvent` |
| Broadcasts | `BCAST_RDR_SAR_STATUS` 31/15 (listeners 13, 15), `BCAST_RDR_SAR_IMAGE` 31/13-14 (13); RT 9 joins `BCAST_FMS_FLIGHT_DATA`'s listeners | section 8 |
| SAR commands | `DSP_RDR_SAR_CMD` (RT 15 SA 22, the layout of `RDR_SAR_MODE_CMD`, relayed since v1.5) no longer proposed: the radar display window's SAR buttons and resolution spinbox | `sar_radar.set_mode`, `rmConfig.xml` |
| Enums | `ground_feature` (NONE, BUILDING, VEHICLE, SHIP, ROAD, RIVER, LAKE, FOREST, URBAN), `change_type` (NONE, ADDITION, REMOVAL, CHANGE) | the plant's scene; `ChangeDetector.process`'s strings |
**Added in v1.14:**

| Field | Now | Taken from |
|---|---|---|
| SMS inputs | RT 17 among the listeners of `BCAST_FMS_STATUS` (nothing new on the bus) | `sms._poll_data()`: `get_tactical_status()` for the mode and the tactical systems |

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
- **Bus load:** a minor frame may use at most **50%** of a real 1 Mbit/s bus. That's 20 µs per word plus about 12 µs of gaps per message. A relayed message is charged as one RT-to-RT transfer per consumer (two command words, the data once, two status words), the way a real controller moves it, with no poll of the controller's own: it hears the data as it crosses (until v1.12 the estimate charged a poll plus a write per consumer, the emulated link's two datagrams). The UDP link is much faster either way, but this keeps the schedule one a real bus could carry.
- **RT-to-RT transfers** carry a producer's message to its consumers, so the data message and the message the consumer receives have the same layout. They are listed in the ICD's **relays** section (`<relay from=… to=…>`): in the minor frame where the bus controller polls `from`, it writes the same words into `to` straight away (on the emulated link; a real controller issues one RT-to-RT command pair), and the consumer's receive message gets no slot of its own. A producer may feed several consumers, one relay each (at most four; `PLANT_SENSORS` goes to the FCS, the FMS and navigation). A consumer may be the broadcast address 31 (v1.7): one transfer that every terminal accepting the subaddress stores, which is how the displays get their data. `icd_tool.py check` enforces that the two share a layout, that `from` transmits periodically and `to` receives, that a message has at most one relay feeding it, and the limit of four consumers.
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
- **Broadcast:** the BC sends the same datagram to every terminal. Nobody responds. A terminal that accepted the subaddress for broadcast (v1.7) stores the data and sets its broadcast-received bit; any other terminal ignores it, so a broadcast can never overwrite a buffer meant for a terminal's own commands. Mode codes (the sync) need no acceptance. Nothing is retried: a broadcast a terminal misses is simply the next one's to make good.

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
- The descriptor and the segment subaddress are polled at the block's rate like any periodic message (every block stream today: every other frame, 25 Hz; `PLANT_TERRAIN` was polled every frame until v1.10). A block of n segments is complete n polls after its publication, so a stream polled every other frame needs a source that publishes no more often than every 2n frames: the four-segment terrain and traffic blocks are published every ten.
- The largest block is 256 × 31 = 7,936 words.
- A consumer accepts a block only when it has every segment of one version and the CRC matches. Otherwise it keeps the previous block, which goes stale normally.

**Block relays.** A relay may connect two block streams. The receiving block has `dir="R"` and is `layout_of` the transmitted one; in the frame where the bus controller polls the producer's descriptor or segment subaddress, it writes the same words into the consumer's, so the consumer reassembles the block exactly as any receiver does. A receive block is never polled. On each side the framing helpers do the work: `bus::BlockPublisher` / `fmofp_bus.block.BlockPublisher` serve a producer's block from its two transmit subaddresses, and `bus::BlockCollector` / `fmofp_bus.block.BlockCollector` collect a consumer's from its two receive subaddresses. The first is `PLANT_TERRAIN` → `RDR_TFR_TERRAIN` (v1.4); `PLANT_TRAFFIC` → `RDR_TRAFFIC` follows in v1.10. A block relay may also go to the broadcast address (v1.7): the displays' copies of `RDR_WX_PRODUCTS` and `RDR_TFR_ELEVATION` are `BCAST_RDR_WX_PRODUCTS` and `BCAST_RDR_TFR_ELEVATION`, one transfer per segment for every listener, and from v1.10 the tracks blocks' are `BCAST_RDR_TGT_TRACKS` and `BCAST_RDR_FUSED_TRACKS`. A listener that misses a segment (nothing is retried on a broadcast) loses that version of the block and draws the next, which the version-and-CRC rule already provides for. A listener need not be a display: from v1.8 the engine computer (RT 3) accepts `BCAST_FMS_FLIGHT_DATA` for its autothrottle and the flight control computer (RT 5) accepts `BCAST_ENG_ENGINE_STATUS` for its fuel efficiency. The broadcast map then has four bands at address 31: 1-10 (radar pictures, flight control), 18-19 (engine), 20-24 (air tracks, from v1.10) and 25-28 (flight management; 28 is the energy maneuverability of v1.10), plus, from v1.12, the AEWC's status and picture at 11 and 16-17, numbers in the displays' own band that only bar the PFD, the SMS and the HUD from listening to them; the collision rule (no listener with a receive message or block of its own on a subaddress it accepts) is what keeps a subsystem's command buffers out of reach of a broadcast.

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
| 6 | Is the Python code still changing? | Freeze everything until the conversion is finished | Port baseline is upstream `0ab9236`; upstream's later commits are merged when they change none of the mirrored logic (section 2) |
| 7 | Fix the section 10 problems in Python? | No, leave them | Section 10 |
| 8 | Legacy `WEATHER` mode? | Keep it unless it's unnecessary | Dropped as unnecessary (section 2) |
| 9 | Undefined values and units? | Full creative liberty | Defined from the code that uses them (section 2) |
| 10 | Heading: true or magnetic? | True | Every heading and yaw field |

## 12. Backlog

Done since v1: the C++ and Python code generators (`gen-cpp`, `gen-py`), the relays section, the bus-load check (in the bus controller), the first partition's inputs and outputs (v1.1), the navigation and flight management partitions' inputs and outputs, the PFD's flight data (`FMS_FLIGHT_DATA`), thrust and gear commands (v1.2), block payload layouts and the weather radar's inputs and products (v1.3), the terrain-following radar's inputs and products, block relays and receive blocks, terrain clearance from the radar (v1.4), the radar display and the PFD on the bus, the radar commands' home (v1.5), the MFD on the bus (v1.6), display data by broadcast (v1.7), the engine management partition with the engines in the plant and the fuel efficiency it gives the flight computer (v1.8), the EICAS on the bus (v1.9), the targeting radar and the track fusion with the air traffic in the plant (v1.10), the TSD on the bus and the targeting radar's displays (v1.11), the AEWC radar (v1.12), the synthetic aperture radar with the ground in the plant (v1.13), the SMS on the bus (v1.14).

- A designated spotlight point for the SAR (`RDR_SAR_MODE_CMD` has none, so SPOTLIGHT stares at the patch abeam when commanded), and a change detector that can report a change (the ported one masks itself, section 2).
- The HUD as a terminal, listening to the broadcasts it needs (the EICAS went on in v1.9, the TSD in v1.11, the SMS in v1.14). The MFD's communications and mission rows, the EICAS's hydraulic, electrical and ECS rows, and the TSD's route, order of battle and radar warning contacts fill in when those terminals exist.
- Retiring `radar_to_display_bridge.py` and the in-process messaging stack from the Python once no display reads them.
- Periodic data messages for each remaining subsystem and display input: hydraulics, electrical and ECS for the EICAS, a stores management computer for the SMS (the stations, the release and delivery modes, the selected station, jettison: today the display's own table), HUD. These come from the display and subsystem inventories, and are added partition by partition as the port reaches them.
- An IFF interrogator: every track's `iff_identity` is UNKNOWN, as the Python's was, so the threat rule only ever uses its RCS-and-speed heuristic.
- Spool-up in the plant's engines: today the measured thrust is the engine computer's command at the engines' next 2 Hz step. A first-order lag would let N1 and the fuel flow trail the command as a real engine's do.
- Afterburner fuel flow: `ENG_THRUST_CMD.afterburner` reaches the plant, whose engine model (the ECU's) has no afterburner, so the flow is 42 x thrust either way.
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
| 1.8-draft | 2026-10-25 | Engine management partition (RT 3): `PLANT_ENGINE` → `ENG_ENGINE_IN`, `ENG_THRUST_CMD` → `PLANT_THRUST_CMD`, `FMS_CONTROL_STATUS` → `ENG_FMS_CONTROL`, `ENG_STATUS`, `ENG_MODE_CMD`, `ENG_CMD`, `ENG_ENGINE_STATUS` (→ `BCAST_ENG_ENGINE_STATUS`, 31/18, listeners MFD and FCS), `ENG_FUEL_STATUS` (→ `BCAST_ENG_FUEL_STATUS`, 31/19); RT 3 added to `BCAST_FMS_FLIGHT_DATA`'s listeners. `FCS_PERF_STATUS.fuel_efficiency` computed. Enums `engine_command`, `engine_select`, `fuel_tank` |
| 1.7-draft | 2026-10-25 | Display data by broadcast: `BCAST_RDR_WX_STATUS`, `BCAST_RDR_WX_SCAN`, `BCAST_RDR_TFR_STATUS`, `BCAST_RDR_TFR_GUIDANCE`, `BCAST_FCS_GCAS_ALERTS`, `BCAST_FCS_PERF_STATUS`, `BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS`, `BCAST_FMS_ATTITUDE_STATUS` and the blocks `BCAST_RDR_WX_PRODUCTS`, `BCAST_RDR_TFR_ELEVATION` at address 31 with `listeners`; the per-display copies `DSP_RDR_*`, `PFD_*`, `MFD_*` removed; terminals accept broadcast subaddresses |
| 1.9-draft | 2026-11-01 | The EICAS (RT 14) on the bus: a listener of `BCAST_ENG_ENGINE_STATUS`, `BCAST_ENG_FUEL_STATUS`, `BCAST_FCS_GCAS_ALERTS`, `BCAST_FCS_PERF_STATUS`, `BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS` and `BCAST_RDR_WX_SCAN`; `RDR_WX_SCAN.max_shear` |
| 1.10-draft | 2026-11-01 | Targeting radar and track fusion: the `PLANT_TRAFFIC` block → `RDR_TRAFFIC`; `RDR_TGT_STATUS` extended; `RDR_TGT_MODE_CMD` with the track filter and the lock request; the `RDR_TGT_TRACKS` block's payload; the `RDR_FUSED_TRACKS` block; `BCAST_RDR_TGT_STATUS`, `BCAST_RDR_TGT_TRACKS`, `BCAST_RDR_FUSED_TRACKS`, `BCAST_FMS_ENERGY`. `PLANT_TERRAIN` and `RDR_TFR_TERRAIN` at 25 Hz. Enums `target_class`, `iff_identity`, `threat`, `lock_status` |
| 1.11-draft | 2026-11-01 | The TSD (RT 16) on the bus: a listener of `BCAST_FMS_FLIGHT_DATA`, `BCAST_FMS_STATUS` and `BCAST_FMS_ATTITUDE_STATUS` beside `BCAST_FMS_ENERGY` and `BCAST_RDR_FUSED_TRACKS`; the MFD's targeting page and the radar display's targeting controls |
| 1.12-draft | 2026-11-01 | AEWC radar: `RDR_AEWC_STATUS` extended; the `RDR_AEWC_TRACKS` block (replacing the TBD `RDR_AEWC_SECTOR`); `BCAST_RDR_AEWC_STATUS`, `BCAST_RDR_AEWC_TRACKS`; `DSP_RDR_AEWC_CMD` no longer proposed. Enums `jam_type`, `ecm_mitigation` |
| 1.13-draft | 2026-11-01 | Synthetic aperture radar: the `PLANT_GROUND` block → `RDR_SAR_SCENE`; `RDR_SAR_STATUS` extended; the `RDR_SAR_IMAGE` block's payload (the last TBD); `BCAST_RDR_SAR_STATUS`, `BCAST_RDR_SAR_IMAGE`; RT 9 a listener of `BCAST_FMS_FLIGHT_DATA`; `DSP_RDR_SAR_CMD` no longer proposed. Enums `ground_feature`, `change_type`. 91 messages, 32 relays, 18 block streams |
| 1.14-draft | 2026-11-02 | The SMS (RT 17) on the bus: a listener of `BCAST_FMS_STATUS`; nothing new on the bus. 91 messages, 32 relays, 18 block streams |
