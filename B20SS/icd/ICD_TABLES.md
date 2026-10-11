# FMOFP 1553B ICD — tables

> Generated from `fmofp_icd.xml` by `tools/icd_tool.py render`. **Do not edit by hand.** ICD version 1.3 (DRAFT, 2026-10-11). Conventions, timing and transport are in [README.md](README.md).

## Terminals

| RT | ID | Name | Status | Source | Note |
|---|---|---|---|---|---|
| 0 | `reserved_0` | Reserved (was 'Unnamed System') | reserved |  |  |
| 1 | `avionics` | Avionics - LRU health, built-in test, FM fitness, software configuration | existing | Systems/avionics/hardwareHealth, Systems/builtInTestSystems, Systems/fmFitness, Systems/configurationManagement | BIT merged here (Alee, 2026-10-10). FM fitness and SW configuration are here too: Alee said to fold them into another terminal, and both are health/maintenance functions |
| 2 | `comms` | Communications | existing | Systems/comms/messaging_service.py |  |
| 3 | `enginemanagement` | Engine management - ECU, fuel, thrust | existing | Systems/engineManagement |  |
| 4 | `enviornmentalcontrolsystem` | Environmental control system | existing | Systems/enviornmentalControlSystem | id keeps the repo's spelling |
| 5 | `flightcontrolsys` | Flight control - FCC, GCAS, performance monitor | existing | Systems/flightControlSys |  |
| 6 | `missionplanning` | Mission planning | existing | Systems/missionPlanning/missionService.py |  |
| 7 | `nav` | Navigation - GPS position solver and GPS/dead-reckoning fusion | existing | Systems/nav: gps/globalPositionSystem.py (GPSReceiver, the solver) and dataFusion/fuseData.py | Ported in v1.2. The INS of Systems/nav/ins was a noise generator whose output nothing used; it is not carried (README section 2) |
| 8 | `powermanagement` | Power management | existing | Systems/powerManagement |  |
| 9 | `radar` | Radar computer - weather, TFR, SAR, targeting, AEWC, track fusion | existing | Systems/radarManagement | The five radars stay one terminal; each radar is a subaddress, as before. The weather radar is ported in v1.3 (SA 1, 6-7, 18-19); the other four are still Python |
| 10 | `sensormanagement` | Sensor management | existing | Systems/sensorManagement |  |
| 11 | `pfd` | Primary flight display | changed | Interfaces/userInterface/displays/pfd.py | Was the single 'Display System' terminal; one process per display means each display needs its own address |
| 12 | `flightmanagementsystem` | Flight management system | existing | Systems/flightManagementSys: flightManagementSystem.py and fmsControl.py | Ported in v1.2. Its attitude comes from the FCS over the bus, as the Python copied it from its own flight control object |
| 13 | `mfd` | Multi-function display | new | Interfaces/userInterface/displays/mfd.py |  |
| 14 | `eicas` | Engine indicating and crew alerting display | new | Interfaces/userInterface/displays |  |
| 15 | `radar_display` | Weather radar display | new | Interfaces/userInterface/displays/radar |  |
| 16 | `tsd` | Tactical situation display | new | Interfaces/userInterface/displays/tsd.py |  |
| 17 | `sms` | Stores management display | new | Interfaces/userInterface/displays |  |
| 18 | `hud` | Head-up display | new | Interfaces/userInterface (HUDContainer) |  |
| 19 | `hydraulics` | Hydraulics | new | Systems/hydraulics |  |
| 20 | `airframe` | Airframe systems | new | Systems/airframeSystemManagement |  |
| 21 | `flightsurfaces` | Flight surfaces | new | Systems/flightSurfaces |  |
| 22 | `defensive` | Defensive systems - RWR, chaff, flare, ECM | new | Systems/defensiveSys |  |
| 23 | `flightdatamonitoring` | Flight data monitoring | new | Systems/flightDataMonitoring |  |
| 27 | `plant` | Plant model - the simulated aircraft and world (test equipment, not aircraft software) | new | Systems/flightControlSys/flightControlComputer (the random-walk flight state), to become a Python plant process | Added in v1.1. Publishes what the sensors would measure. Each quantity moves to its real producer as that subsystem is ported (terrain clearance to the radar, engine data to the engine terminal) |

RT 31 is broadcast. Unused: 24, 25, 26, 28, 29, 30.

## Message summary

| ID | RT | SA | Dir | Words | Rate (Hz) | Title | Status |
|---|---|---|---|---|---|---|---|
| [`FCS_MODE_CMD`](#fcs_mode_cmd) | 5 | 10 | R | 1 | aperiodic | Flight control mode command | DRAFT |
| [`FCS_STATUS`](#fcs_status) | 5 | 10 | T | 2 | 5 | Flight control status | DRAFT |
| [`FCS_SURFACE_CMD`](#fcs_surface_cmd) | 5 | 11 | R | 2 | aperiodic | Control surface command | DRAFT |
| [`FCS_SURFACE_STATUS`](#fcs_surface_status) | 5 | 11 | T | 7 | 10 | Control surface positions | DRAFT |
| [`FCS_ATTITUDE`](#fcs_attitude) | 5 | 12 | T | 4 | 50 | Aircraft attitude | DRAFT |
| [`FCS_SENSORS`](#fcs_sensors) | 5 | 13 | R | 12 | 50 | Sensed flight state as received by the flight control computer | DRAFT |
| [`FCS_GCAS_ALERTS`](#fcs_gcas_alerts) | 5 | 14 | T | 4 | 5 | Ground collision avoidance alerts | DRAFT |
| [`FCS_PERF_STATUS`](#fcs_perf_status) | 5 | 15 | T | 4 | 1 | Flight envelope monitoring | DRAFT |
| [`NAV_SENSORS`](#nav_sensors) | 7 | 13 | R | 12 | 50 | Sensed flight state, for navigation | DRAFT |
| [`NAV_GPS`](#nav_gps) | 7 | 14 | R | 25 | 1 | GPS pseudoranges, for navigation | DRAFT |
| [`NAV_SOLUTION`](#nav_solution) | 7 | 15 | T | 8 | 10 | Navigation solution | DRAFT |
| [`RDR_WX_MODE_CMD`](#rdr_wx_mode_cmd) | 9 | 1 | R | 3 | aperiodic | Weather radar mode command | DRAFT |
| [`RDR_WX_STATUS`](#rdr_wx_status) | 9 | 1 | T | 2 | 5 | Weather radar status | DRAFT |
| [`RDR_TFR_MODE_CMD`](#rdr_tfr_mode_cmd) | 9 | 2 | R | 2 | aperiodic | Terrain-following radar mode command | DRAFT |
| [`RDR_TFR_STATUS`](#rdr_tfr_status) | 9 | 2 | T | 2 | 5 | Terrain-following radar status | DRAFT |
| [`RDR_SAR_MODE_CMD`](#rdr_sar_mode_cmd) | 9 | 3 | R | 2 | aperiodic | Synthetic aperture radar mode command | DRAFT |
| [`RDR_SAR_STATUS`](#rdr_sar_status) | 9 | 3 | T | 2 | 5 | Synthetic aperture radar status | DRAFT |
| [`RDR_TGT_MODE_CMD`](#rdr_tgt_mode_cmd) | 9 | 4 | R | 2 | aperiodic | Targeting radar mode command | DRAFT |
| [`RDR_TGT_STATUS`](#rdr_tgt_status) | 9 | 4 | T | 2 | 5 | Targeting radar status | DRAFT |
| [`RDR_AEWC_MODE_CMD`](#rdr_aewc_mode_cmd) | 9 | 5 | R | 4 | aperiodic | AEWC radar mode command | DRAFT |
| [`RDR_AEWC_STATUS`](#rdr_aewc_status) | 9 | 5 | T | 2 | 5 | AEWC radar status | DRAFT |
| [`RDR_WX_SCAN`](#rdr_wx_scan) | 9 | 18 | T | 10 | 5 | Weather radar scan summary | DRAFT |
| [`RDR_WX_WEATHER`](#rdr_wx_weather) | 9 | 19 | R | 32 | 1 | Weather truth, for the radar's sensor model | DRAFT |
| [`DSP_PFD_MODE_CMD`](#dsp_pfd_mode_cmd) | 11 | 11 | R | 1 | aperiodic | Primary flight display mode command | DRAFT |
| [`DSP_PFD_STATUS`](#dsp_pfd_status) | 11 | 11 | T | 2 | 1 | Primary flight display status | DRAFT |
| [`FMS_ATTITUDE_IN`](#fms_attitude_in) | 12 | 12 | R | 4 | 50 | Measured attitude, from the FCS | DRAFT |
| [`FMS_SENSORS`](#fms_sensors) | 12 | 13 | R | 12 | 50 | Sensed flight state, for the FMS | DRAFT |
| [`FMS_NAV_SOLUTION`](#fms_nav_solution) | 12 | 14 | R | 8 | 10 | Navigation solution, for the FMS | DRAFT |
| [`FMS_MODE_CMD`](#fms_mode_cmd) | 12 | 15 | R | 1 | aperiodic | Flight management mode command | DRAFT |
| [`FMS_STATUS`](#fms_status) | 12 | 15 | T | 3 | 5 | Flight management status | DRAFT |
| [`FMS_ATTITUDE_CMD`](#fms_attitude_cmd) | 12 | 16 | R | 7 | aperiodic | Attitude command | DRAFT |
| [`FMS_ATTITUDE_STATUS`](#fms_attitude_status) | 12 | 16 | T | 8 | 10 | Attitude command status | DRAFT |
| [`FMS_NAV_UPDATE_CMD`](#fms_nav_update_cmd) | 12 | 17 | R | 5 | aperiodic | Autopilot target command | DRAFT |
| [`FMS_NAV_UPDATE_STATUS`](#fms_nav_update_status) | 12 | 17 | T | 6 | 5 | Autopilot target status | DRAFT |
| [`FMS_MANEUVER_CMD`](#fms_maneuver_cmd) | 12 | 18 | R | 3 | aperiodic | Maneuver command | DRAFT |
| [`FMS_MANEUVER_STATUS`](#fms_maneuver_status) | 12 | 18 | T | 2 | 5 | Maneuver status | DRAFT |
| [`FMS_FLIGHT_DATA`](#fms_flight_data) | 12 | 19 | T | 21 | 10 | FMS flight data | DRAFT |
| [`FMS_FLIGHT_MODE_CMD`](#fms_flight_mode_cmd) | 12 | 20 | R | 1 | aperiodic | Flight mode command | DRAFT |
| [`FMS_CONTROL_CMD`](#fms_control_cmd) | 12 | 21 | R | 2 | aperiodic | Thrust and landing gear command | DRAFT |
| [`FMS_CONTROL_STATUS`](#fms_control_status) | 12 | 21 | T | 3 | 5 | Thrust and landing gear status | DRAFT |
| [`FMS_ENERGY`](#fms_energy) | 12 | 22 | T | 6 | 1 | Energy maneuverability | DRAFT |
| [`DSP_MFD_MODE_CMD`](#dsp_mfd_mode_cmd) | 13 | 12 | R | 1 | aperiodic | Multi-function display mode command | DRAFT |
| [`DSP_MFD_STATUS`](#dsp_mfd_status) | 13 | 12 | T | 2 | 1 | Multi-function display status | DRAFT |
| [`MFD_RDR_WX_CMD`](#mfd_rdr_wx_cmd) | 13 | 20 | T | 3 | 10 | MFD command to the weather radar | PROPOSED |
| [`MFD_RDR_TFR_CMD`](#mfd_rdr_tfr_cmd) | 13 | 21 | T | 2 | 10 | MFD command to the terrain-following radar | PROPOSED |
| [`MFD_RDR_SAR_CMD`](#mfd_rdr_sar_cmd) | 13 | 22 | T | 2 | 10 | MFD command to the synthetic aperture radar | PROPOSED |
| [`MFD_RDR_TGT_CMD`](#mfd_rdr_tgt_cmd) | 13 | 23 | T | 2 | 10 | MFD command to the targeting radar | PROPOSED |
| [`MFD_RDR_AEWC_CMD`](#mfd_rdr_aewc_cmd) | 13 | 24 | T | 4 | 10 | MFD command to the aewc radar | PROPOSED |
| [`DSP_EICAS_MODE_CMD`](#dsp_eicas_mode_cmd) | 14 | 13 | R | 1 | aperiodic | EICAS mode command | DRAFT |
| [`DSP_EICAS_STATUS`](#dsp_eicas_status) | 14 | 13 | T | 2 | 1 | EICAS status | DRAFT |
| [`DSP_RADAR_MODE_CMD`](#dsp_radar_mode_cmd) | 15 | 14 | R | 1 | aperiodic | Weather radar display mode command | DRAFT |
| [`DSP_RADAR_STATUS`](#dsp_radar_status) | 15 | 14 | T | 2 | 1 | Weather radar display status | DRAFT |
| [`DSP_TSD_MODE_CMD`](#dsp_tsd_mode_cmd) | 16 | 15 | R | 1 | aperiodic | Tactical situation display mode command | DRAFT |
| [`DSP_TSD_STATUS`](#dsp_tsd_status) | 16 | 15 | T | 2 | 1 | Tactical situation display status | DRAFT |
| [`DSP_SMS_MODE_CMD`](#dsp_sms_mode_cmd) | 17 | 16 | R | 1 | aperiodic | Stores management display mode command | DRAFT |
| [`DSP_SMS_STATUS`](#dsp_sms_status) | 17 | 16 | T | 2 | 1 | Stores management display status | DRAFT |
| [`DSP_HUD_MODE_CMD`](#dsp_hud_mode_cmd) | 18 | 17 | R | 1 | aperiodic | Head-up display mode command | DRAFT |
| [`DSP_HUD_STATUS`](#dsp_hud_status) | 18 | 17 | T | 2 | 1 | Head-up display status | DRAFT |
| [`PLANT_SENSORS`](#plant_sensors) | 27 | 1 | T | 12 | 50 | Sensed flight state from the plant | DRAFT |
| [`PLANT_GPS`](#plant_gps) | 27 | 2 | T | 25 | 1 | GPS pseudoranges from the plant | DRAFT |
| [`PLANT_WEATHER`](#plant_weather) | 27 | 3 | T | 32 | 1 | Weather truth from the plant | DRAFT |
| [`TERM_CMD`](#term_cmd) | all | 29 | R | 1 | aperiodic | Terminal command | DRAFT |
| [`TERM_STATUS`](#term_status) | all | 29 | T | 5 | 1 | Terminal status | DRAFT |

## Block transfers

The descriptor and segment framing is in README.md section 8. A block's payload is its header, one count word per section, then every section's records in order; the layouts are under [Block payloads](#block-payloads).

| ID | RT | Descriptor SA | Segment SA | Rate (Hz) | Title | Payload |
|---|---|---|---|---|---|---|
| `RDR_WX_PRODUCTS` | 9 | 6 | 7 | 50 | Weather radar products | [header 2 words + 6 sections, at most 548 words](#rdr_wx_products) |
| `RDR_TGT_TRACKS` | 9 | 10 | 11 | 10 | Targeting radar tracks | TBD-v2 |
| `RDR_AEWC_SECTOR` | 9 | 12 | 13 | 10 | AEWC sector scan | TBD-v2 |
| `RDR_TFR_ELEVATION` | 9 | 14 | 15 | 10 | TFR elevation profile | TBD-v2 |
| `RDR_SAR_IMAGE` | 9 | 16 | 17 | 1 | SAR image | TBD-v2 |

## Relays (RT-to-RT transfers)

In the minor frame where the bus controller polls `from`, it writes the same words into `to` straight away. This is how a producer's data reaches its consumers and how a display's command reaches the terminal it is for.

| From | To | Words | Rate (Hz) |
|---|---|---|---|
| [`PLANT_SENSORS`](#plant_sensors) | [`FCS_SENSORS`](#fcs_sensors) | 12 | 50 |
| [`PLANT_SENSORS`](#plant_sensors) | [`FMS_SENSORS`](#fms_sensors) | 12 | 50 |
| [`PLANT_SENSORS`](#plant_sensors) | [`NAV_SENSORS`](#nav_sensors) | 12 | 50 |
| [`PLANT_GPS`](#plant_gps) | [`NAV_GPS`](#nav_gps) | 25 | 1 |
| [`PLANT_WEATHER`](#plant_weather) | [`RDR_WX_WEATHER`](#rdr_wx_weather) | 32 | 1 |
| [`FCS_ATTITUDE`](#fcs_attitude) | [`FMS_ATTITUDE_IN`](#fms_attitude_in) | 4 | 50 |
| [`NAV_SOLUTION`](#nav_solution) | [`FMS_NAV_SOLUTION`](#fms_nav_solution) | 8 | 10 |
| [`MFD_RDR_WX_CMD`](#mfd_rdr_wx_cmd) | [`RDR_WX_MODE_CMD`](#rdr_wx_mode_cmd) | 3 | 10 |
| [`MFD_RDR_TFR_CMD`](#mfd_rdr_tfr_cmd) | [`RDR_TFR_MODE_CMD`](#rdr_tfr_mode_cmd) | 2 | 10 |
| [`MFD_RDR_SAR_CMD`](#mfd_rdr_sar_cmd) | [`RDR_SAR_MODE_CMD`](#rdr_sar_mode_cmd) | 2 | 10 |
| [`MFD_RDR_TGT_CMD`](#mfd_rdr_tgt_cmd) | [`RDR_TGT_MODE_CMD`](#rdr_tgt_mode_cmd) | 2 | 10 |
| [`MFD_RDR_AEWC_CMD`](#mfd_rdr_aewc_cmd) | [`RDR_AEWC_MODE_CMD`](#rdr_aewc_mode_cmd) | 4 | 10 |

## Mode codes

| Code | Name | Dir | Data words | Broadcast | Use |
|---|---|---|---|---|---|
| 2 | TRANSMIT_STATUS_WORD | T | 0 | no | BC health poll; also how the BC reads the message-error bit after a no-response |
| 3 | INITIATE_SELF_TEST | T | 0 | yes | Starts initiated BIT; results in the BIT word |
| 6 | INHIBIT_TERMINAL_FLAG | T | 0 | yes | Terminal flag bit forced to 0 in the status word |
| 7 | OVERRIDE_INHIBIT_TERMINAL_FLAG | T | 0 | yes | Cancels mode code 6 |
| 8 | RESET_REMOTE_TERMINAL | T | 0 | yes | Terminal re-initialises: state INITIALIZING, latched faults cleared |
| 16 | TRANSMIT_VECTOR_WORD | T | 1 | no | After a service request: data word = SA (bits 4-0) the terminal wants serviced |
| 17 | SYNCHRONIZE_WITH_DATA | R | 1 | yes | Sent to broadcast RT 31 at the start of every minor frame. Data word: bits 15-6 major frame count (mod 1024), bits 5-0 minor frame 0-49 |
| 18 | TRANSMIT_LAST_COMMAND | T | 1 | no | Returns the last command word received |
| 19 | TRANSMIT_BIT_WORD | T | 1 | no | Returns the BIT word (same layout as TERM_STATUS word 2) |

## Enumerations

### `terminal_state` (4 bits)

Lifecycle and health state shared by every terminal. Values 0-8 keep their MODE_VALUE_MAP numbers; INITIALIZING moves from -1 to 15.

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | STANDBY |  |  |
| 1 | NORMAL |  |  |
| 2 | DEGRADED |  |  |
| 3 | TEST |  |  |
| 4 | MAINTENANCE |  |  |
| 5 | EMERGENCY |  |  |
| 6 | FAILURE |  |  |
| 7 | RECOVERY |  |  |
| 8 | CALIBRATION |  |  |
| 15 | INITIALIZING | -1 |  |

### `weather_mode` (4 bits)

Weather radar operating mode (radar_enums.weather_radarMode). The legacy WEATHER value (6) is dropped: no radar implements it and nothing sends it. Numbered per radar from 1; legacy = old global number.

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  | No operating mode selected (state is not NORMAL) |
| 1 | SURVEILLANCE | 10 |  |
| 2 | MAPPING | 11 |  |
| 3 | TURBULENCE | 12 |  |
| 4 | WINDSHEAR | 13 |  |
| 5 | PRECIPITATION | 14 |  |

### `tfr_mode` (4 bits)

TFR operating mode (radar_enums.tfr_radarMode). Numbered per radar from 1; legacy = old global number.

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  | No operating mode selected (state is not NORMAL) |
| 1 | TFR_SEARCH | 20 |  |
| 2 | TFR_TRACK | 21 |  |
| 3 | TFR_ACTIVE | 22 |  |
| 4 | TERRAIN_FOLLOWING | 23 |  |
| 5 | OBSTACLE_AVOIDANCE | 24 |  |
| 6 | GROUND_MAPPING | 25 |  |

### `sar_mode` (4 bits)

SAR operating mode (radar_enums.sar_radarMode). Numbered per radar from 1; legacy = old global number.

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  | No operating mode selected (state is not NORMAL) |
| 1 | STRIPMAP | 30 |  |
| 2 | SPOTLIGHT | 31 |  |
| 3 | SCANSAR | 32 |  |
| 4 | INTERFEROMETRIC | 33 |  |
| 5 | DOPPLER_BEAM | 34 |  |

### `targeting_mode` (4 bits)

Targeting radar operating mode (radar_enums.targeting_radarMode). Numbered per radar from 1; legacy = old global number.

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  | No operating mode selected (state is not NORMAL) |
| 1 | TARGET_SEARCH | 40 |  |
| 2 | TARGET_TRACK | 41 |  |
| 3 | LOCK | 42 |  |
| 4 | TERRAIN_AVOIDANCE | 43 |  |
| 5 | GROUND_MAPPING | 25 |  |

### `aewc_mode` (4 bits)

AEWC radar operating mode (radar_enums.aewc_radarMode). Numbered per radar from 1; legacy = old global number.

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  | No operating mode selected (state is not NORMAL) |
| 1 | AEWC_SEARCH | 50 |  |
| 2 | AEWC_SURVEILLANCE | 51 |  |
| 3 | SECTOR_SCAN | 52 |  |
| 4 | STEALTH_DETECTION | 53 |  |
| 5 | ELECTRONIC_PROTECTION | 54 |  |
| 6 | AEWC_TRACK | 55 |  |
| 7 | GROUND_MAPPING | 25 |  |

### `display_mode` (4 bits)

Display lighting mode (displays/base_display.DisplayMode)

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NOT_SET |  |  |
| 1 | DAY |  |  |
| 2 | NIGHT |  |  |
| 3 | NVG |  |  |

### `ssm` (2 bits)

Sign/status matrix (ARINC 429 BNR convention). All-zero reads as FAILURE_WARNING, so an uninitialised word is never taken for good data.

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | FAILURE_WARNING |  |  |
| 1 | NO_COMPUTED_DATA |  |  |
| 2 | FUNCTIONAL_TEST |  |  |
| 3 | NORMAL |  |  |

### `cmd_status` (4 bits)

Outcome of a command that does more than change mode

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  | No command received yet |
| 1 | ACCEPTED |  | Accepted and in progress |
| 2 | COMPLETED |  | Done |
| 3 | REJECTED_INVALID |  | A parameter was out of range or unknown |
| 4 | REJECTED_STATE |  | Not allowed in the terminal's current state or mode |
| 5 | FAILED |  | Accepted but could not be completed |
| 6 | NOT_IMPLEMENTED |  | Recognised but not implemented (for example BARREL_ROLL in fmsControl.py) |

### `control_surface` (4 bits)

Control surfaces, from fmsControl.set_control_surfaces()

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  |  |
| 1 | AILERONS |  |  |
| 2 | ELEVATOR |  |  |
| 3 | RUDDER |  |  |
| 4 | FLAPS |  |  |
| 5 | SPOILERS |  |  |

### `track_filter` (4 bits)

Which tracks the targeting radar reports; classes from radar_data_fusion (HOSTILE, FRIENDLY, UNKNOWN)

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | ALL |  |  |
| 1 | HOSTILE |  |  |
| 2 | FRIENDLY |  |  |
| 3 | UNKNOWN |  |  |

### `maneuver` (4 bits)

Maneuvers fmsControl.execute_maneuver() knows

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  |  |
| 1 | BREAK_RIGHT |  |  |
| 2 | BREAK_LEFT |  |  |
| 3 | BARREL_ROLL |  | fmsControl.py reports it as not implemented |
| 4 | DEFENSIVE_SPLIT_S |  |  |
| 5 | MAXIMUM_CLIMB |  |  |
| 6 | DIVE |  |  |

### `gear_position` (2 bits)

Landing gear position, from the plant's gear state (flightControlComputer landing_gear)

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | UNKNOWN |  | No valid gear indication |
| 1 | UP |  |  |
| 2 | DOWN |  |  |
| 3 | IN_TRANSIT |  |  |

### `gcas_status` (2 bits)

Overall ground collision avoidance state (groundCollisionAvoidanceSys.get_data: ARMED, CAUTION, WARNING)

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | OFF |  | Not running, or no valid inputs |
| 1 | ARMED |  | Running, no alert |
| 2 | CAUTION |  | Severity-2 alert active (ALT LOW) |
| 3 | WARNING |  | Severity-1 alert active (PULLUP, SINK RATE or TERRAIN) |

### `fms_flight_mode` (4 bits)

FMS flight mode (fmsControl.available_modes), numbered as flightManagementSystem.send_fms_data did. Each selects a flight profile - max bank 30/80/45/60/90 deg, max pitch 20/60/30/45/85 deg, max g 2.5/9/5/7/9.5, max AoA 15/25/20/22/30 deg, engine power 0.7/1.0/0.6/0.9/1.0, turn rate limit 3/20/5/15/25 deg/s - for NORMAL/COMBAT/STEALTH/TRAINING/EMERGENCY

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NORMAL |  |  |
| 1 | COMBAT |  |  |
| 2 | STEALTH |  |  |
| 3 | TRAINING |  |  |
| 4 | EMERGENCY |  |  |

### `countermeasures_state` (2 bits)

fmsControl tactical_systems countermeasures, set by the flight mode

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | STANDBY |  |  |
| 1 | READY |  |  |
| 2 | PASSIVE |  |  |
| 3 | ACTIVE |  |  |

### `targeting_state` (2 bits)

fmsControl tactical_systems targeting, set by the flight mode

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | STANDBY |  |  |
| 1 | ACTIVE |  |  |
| 2 | PASSIVE |  |  |
| 3 | READY |  |  |

### `weapons_state` (2 bits)

fmsControl tactical_systems weapons, set by the flight mode

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | SAFE |  |  |
| 1 | ARMED |  |  |
| 2 | READY |  |  |

### `precip_type` (3 bits)

Precipitation type in a weather radar product. The point product (precipitation_data_generator_sync) classifies by rate: HAIL above 50 mm/h, MIXED above 25, RAIN above 10, else RAIN or SNOW; the area product (precipitation_analysis) by reflectivity: HAIL above 55 dBZ, HEAVY_RAIN above 45, else RAIN

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  | Not classified |
| 1 | RAIN |  |  |
| 2 | SNOW |  |  |
| 3 | MIXED |  |  |
| 4 | HAIL |  |  |
| 5 | HEAVY_RAIN |  |  |

### `shear_severity` (2 bits)

Wind shear event severity (weather_processor.WindShearProcessor): LOW from 15 kt/nm, MODERATE above 20, SEVERE above 30

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  |  |
| 1 | LOW |  |  |
| 2 | MODERATE |  |  |
| 3 | SEVERE |  |  |

### `turbulence_category` (3 bits)

Turbulence category by eddy dissipation rate proxy (weather_processor.TurbulenceProcessor, the FAA AC 120-88A bands): LIGHT from 0.1, MODERATE from 0.2, SEVERE from 0.4, EXTREME from 0.6

| Value | Name | Legacy value | Note |
|---|---|---|---|
| 0 | NONE |  |  |
| 1 | LIGHT |  |  |
| 2 | MODERATE |  |  |
| 3 | SEVERE |  |  |
| 4 | EXTREME |  |  |

## Messages

### TERM_STATUS

**Terminal status** — every terminal · SA 29 · transmit (terminal to bus) · 5 words · 1 Hz

Replaces: `systemStatusRequest`, `systemStatusResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last TERM_CMD was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |
| 2 | 15 | `bit_in_progress` | bool |  | Same layout as the BIT word (mode code 19) |
| 2 | 14 | `bit_failed` | bool |  | Latched until RESET_REMOTE_TERMINAL |
| 2 | 13-8 | `spare_2` | spare |  |  |
| 2 | 7-0 | `fault_count` | uint | raw counts | Faults since reset, saturates at 255 |
| 3 | 15-8 | `icd_major` | uint | raw counts | ICD version this terminal was built against |
| 3 | 7-0 | `icd_minor` | uint | raw counts |  |
| 4 | 15-0 | `last_sync` | uint | raw counts | Data word of the last SYNCHRONIZE_WITH_DATA received |

### TERM_CMD

**Terminal command** — every terminal · SA 29 · receive (bus to terminal) · 1 words · aperiodic

The legacy command_parameter word is gone: nothing in the code consumed it.

Replaces: `systemCommand`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### RDR_WX_STATUS

**Weather radar status** — RT 9 · SA 1 · transmit (terminal to bus) · 2 words · 5 Hz

Replaces: `weather_radarStatusRequest`, `weather_radarStatusResponse`, `weather_radarModeChangeResponse`, `radarStatusRequest`, `radarStatusResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `weather_mode` | Operating mode; NONE unless state is NORMAL |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14 | `mode_transition` | bool |  | 1 = a commanded mode change is still in progress |
| 1 | 13-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### RDR_WX_MODE_CMD

**Weather radar mode command** — RT 9 · SA 1 · receive (bus to terminal) · 3 words · aperiodic

The legacy scan_type had no meaning anywhere in the code; range and tilt are the settings weather_radar.py uses.

Replaces: `weather_radarCommand`, `weather_radarModeChangeRequest`, `weather_radarVILRequest`, `weather_radarPrecipitationRequest`, `radarCommand`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `weather_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |
| 1 | 15-0 | `range_km` | uint | LSB 1 km, range 10 to 200 | Scan range; limits from rmConfig.xml weather_radar scan_range (WeatherRadar.set_range takes km) |
| 2 | 15-0 | `tilt` | sint | LSB 180/32768 deg, range 0 to 90 | Antenna tilt, up positive; limits from rmConfig.xml weather_radar tilt_range |

### RDR_WX_SCAN

**Weather radar scan summary** — RT 9 · SA 18 · transmit (terminal to bus) · 10 words · 5 Hz

What the last completed scan found, for the display's header and for anyone who does not reassemble the RDR_WX_PRODUCTS block. While NORMAL in an operating mode the radar sweeps the full circle over 60 minor frames (1.2 s), six azimuth degrees per frame, rendering the volume from the weather truth it latched when the sweep began; the products of a scan are published as one block when the scan completes, and products_version is the block version carrying them. The counts are what was detected before the block's record limits were applied.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-8 | `scan_id` | uint | raw counts | Id of the last completed scan, increments mod 256 |
| 0 | 7-4 | `mode` | enum | enum `weather_mode` | Mode the scan was made in; NONE when no scan has completed since the mode was left |
| 0 | 3 | `scanning` | bool |  | 1 = a scan is in progress |
| 0 | 2-0 | `max_turbulence` | enum | enum `turbulence_category` | Strongest turbulence category found (TURBULENCE mode) |
| 1 | 15 | `microburst` | bool |  | 1 = a wind shear event with microburst divergence was found (WINDSHEAR mode) |
| 1 | 14-12 | `spare_1` | spare |  |  |
| 1 | 11-8 | `elevations` | uint | raw counts | Elevation angles in the scan's volume coverage pattern: 4 for SURVEILLANCE, 7 for MAPPING and PRECIPITATION, 1 for TURBULENCE and WINDSHEAR |
| 1 | 7-0 | `products_version` | uint | raw counts | RDR_WX_PRODUCTS block version carrying this scan's products |
| 2 | 15-0 | `range_km` | uint | LSB 1 km, range 10 to 200 | Instrumented range in effect; range bins beyond it are not scanned |
| 3 | 15-0 | `tilt` | sint | LSB 180/32768 deg, range 0 to 90 | Antenna tilt in effect, reported as commanded; the sensor model has no antenna to tilt |
| 4 | 15-0 | `max_reflectivity` | sint | LSB 1/128 dBZ, range -20 to 65 | Strongest return in the volume |
| 5 | 15-8 | `storm_cells` | uint | raw counts | Storm cells the tracker holds (saturates at 255) |
| 5 | 7-0 | `precip_areas` | uint | raw counts | Precipitation areas the analyzer found (saturates at 255) |
| 6 | 15-8 | `windshear_events` | uint | raw counts | Wind shear events detected (saturates at 255) |
| 6 | 7-0 | `turbulence_cells` | uint | raw counts | Turbulence cells detected (saturates at 255) |
| 7 | 15-8 | `vil_points` | uint | raw counts | VIL points in the block |
| 7 | 7-0 | `precip_points` | uint | raw counts | Precipitation points in the block |
| 8 | 15-0 | `vil_max` | uint | LSB 1/4 kg/m2, range 0 to 16383.75 | Largest VIL in the field |
| 9 | 15-0 | `scan_frame` | uint | raw counts | The radar's own frame count (mod 65536) when the scan completed |

### RDR_WX_WEATHER

**Weather truth, for the radar's sensor model** — RT 9 · SA 19 · receive (bus to terminal) · 32 words · 1 Hz

PLANT_WEATHER relayed to the radar. The radar renders what its antenna would see from it (reflectivity_simulator._render_storm_cells, _render_stratiform_areas and _add_ground_clutter) and processes that volume; nothing else about the weather is known to it.

Same layout as [`PLANT_WEATHER`](#plant_weather).

### RDR_TFR_STATUS

**Terrain-following radar status** — RT 9 · SA 2 · transmit (terminal to bus) · 2 words · 5 Hz

Replaces: `tfr_radarStatusRequest`, `tfr_radarStatusResponse`, `tfr_radarModeChangeResponse`, `radarStatusRequest`, `radarStatusResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `tfr_mode` | Operating mode; NONE unless state is NORMAL |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14 | `mode_transition` | bool |  | 1 = a commanded mode change is still in progress |
| 1 | 13-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### RDR_TFR_MODE_CMD

**Terrain-following radar mode command** — RT 9 · SA 2 · receive (bus to terminal) · 2 words · aperiodic

The legacy scan_type had no meaning anywhere in the code; set clearance is the setting a TFR is flown with.

Replaces: `tfr_radarCommand`, `tfr_radarModeChangeRequest`, `tfr_radarElevationDataRequest`, `radarCommand`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `tfr_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |
| 1 | 15-0 | `set_clearance` | uint | LSB 1 ft, range 30 to 5000 | Terrain-following set clearance height; limits from rmConfig.xml tfr_radar altitude_range, read as feet |

### RDR_SAR_STATUS

**Synthetic aperture radar status** — RT 9 · SA 3 · transmit (terminal to bus) · 2 words · 5 Hz

Replaces: `sar_radarStatusRequest`, `sar_radarStatusResponse`, `sar_radarModeChangeResponse`, `radarStatusRequest`, `radarStatusResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `sar_mode` | Operating mode; NONE unless state is NORMAL |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14 | `mode_transition` | bool |  | 1 = a commanded mode change is still in progress |
| 1 | 13-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### RDR_SAR_MODE_CMD

**Synthetic aperture radar mode command** — RT 9 · SA 3 · receive (bus to terminal) · 2 words · aperiodic

The legacy image_type is dropped: the SAR operating mode (STRIPMAP, SPOTLIGHT, ...) already selects the image type.

Replaces: `sar_radarCommand`, `sar_radarModeChangeRequest`, `sar_radarImageRequest`, `radarCommand`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `sar_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |
| 1 | 15-0 | `resolution` | uint | LSB 1/100 m, range 0.1 to 10 | Image resolution per pixel; sar_radar.py default 1.0 m, limits from rmConfig.xml sar_radar resolution |

### RDR_TGT_STATUS

**Targeting radar status** — RT 9 · SA 4 · transmit (terminal to bus) · 2 words · 5 Hz

Replaces: `targeting_radarStatusRequest`, `targeting_radarStatusResponse`, `targeting_radarModeChangeResponse`, `radarStatusRequest`, `radarStatusResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `targeting_mode` | Operating mode; NONE unless state is NORMAL |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14 | `mode_transition` | bool |  | 1 = a commanded mode change is still in progress |
| 1 | 13-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### RDR_TGT_MODE_CMD

**Targeting radar mode command** — RT 9 · SA 4 · receive (bus to terminal) · 2 words · aperiodic

Replaces: `targeting_radarCommand`, `targeting_radarModeChangeRequest`, `targeting_radarTrackRequest`, `radarCommand`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `targeting_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |
| 1 | 15-4 | `spare_1` | spare |  |  |
| 1 | 3-0 | `track_filter` | enum | enum `track_filter` | Which tracks to report (legacy track_type) |

### RDR_AEWC_STATUS

**AEWC radar status** — RT 9 · SA 5 · transmit (terminal to bus) · 2 words · 5 Hz

Replaces: `aewc_radarStatusRequest`, `aewc_radarStatusResponse`, `aewc_radarModeChangeResponse`, `radarStatusRequest`, `radarStatusResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `aewc_mode` | Operating mode; NONE unless state is NORMAL |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14 | `mode_transition` | bool |  | 1 = a commanded mode change is still in progress |
| 1 | 13-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### RDR_AEWC_MODE_CMD

**AEWC radar mode command** — RT 9 · SA 5 · receive (bus to terminal) · 4 words · aperiodic

The legacy sector_type and sector_parameters became these three fields, taken from the sector scan request in Interfaces/predefinedMessages/aewc_radar_messages.py.

Replaces: `aewc_radarCommand`, `aewc_radarModeChangeRequest`, `aewc_radarSectorScanRequest`, `radarCommand`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `aewc_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |
| 1 | 15-0 | `azimuth_start` | sint | LSB 180/32768 deg, range -180 to 179.9945 | Sector start, relative to the nose, right positive (aewc_radar_messages.py default 0) |
| 2 | 15-0 | `azimuth_end` | sint | LSB 180/32768 deg, range -180 to 179.9945 | Sector end (default 90) |
| 3 | 15-0 | `elevation` | sint | LSB 180/32768 deg, range -5 to 85 | Up positive (default 0); limits from rmConfig.xml aewc_radar tilt_range |

### FCS_STATUS

**Flight control status** — RT 5 · SA 10 · transmit (terminal to bus) · 2 words · 5 Hz

Replaces: `flight_control_systemStatusRequest`, `flight_control_systemStatusResponse`, `flight_control_systemModeChangeResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14 | `mode_transition` | bool |  | 1 = a commanded mode change is still in progress |
| 1 | 13-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### FCS_MODE_CMD

**Flight control mode command** — RT 5 · SA 10 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `flight_control_systemCommand`, `flight_control_systemModeChangeRequest`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### FCS_SURFACE_CMD

**Control surface command** — RT 5 · SA 11 · receive (bus to terminal) · 2 words · aperiodic

One surface per command. Positions are percent of travel, as fmsControl.py uses; the old FCS handler's -1..+1 scaling is replaced.

Replaces: `flight_control_systemControlInputRequest`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `spare_0` | spare |  |  |
| 0 | 11-8 | `surface` | enum | enum `control_surface` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts |  |
| 1 | 15-0 | `position` | sint | LSB 1/100 %, range -100 to 100 | Percent of full travel. AILERONS + = roll right, ELEVATOR + = nose up, RUDDER + = nose right; FLAPS 0-100 % = 0-30 deg; SPOILERS 0-100 % deployed (fmsControl.set_control_surfaces ranges) |

### FCS_SURFACE_STATUS

**Control surface positions** — RT 5 · SA 11 · transmit (terminal to bus) · 7 words · 10 Hz

Every surface, every cycle, instead of one reply per request.

Replaces: `flight_control_systemControlInputResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `ailerons_ssm` | ssm | validity of `ailerons` |  |
| 0 | 13-12 | `elevator_ssm` | ssm | validity of `elevator` |  |
| 0 | 11-10 | `rudder_ssm` | ssm | validity of `rudder` |  |
| 0 | 9-8 | `flaps_ssm` | ssm | validity of `flaps` |  |
| 0 | 7-6 | `spoilers_ssm` | ssm | validity of `spoilers` |  |
| 0 | 5-0 | `spare_0` | spare |  |  |
| 1 | 15-12 | `last_cmd_status` | enum | enum `cmd_status` | Outcome of the last FCS_SURFACE_CMD |
| 1 | 11-8 | `spare_1` | spare |  |  |
| 1 | 7-0 | `last_cmd_seq` | uint | raw counts |  |
| 2 | 15-0 | `ailerons` | sint | LSB 1/100 %, range -100 to 100 | Percent of full travel. AILERONS + = roll right, ELEVATOR + = nose up, RUDDER + = nose right; FLAPS 0-100 % = 0-30 deg; SPOILERS 0-100 % deployed (fmsControl.set_control_surfaces ranges) |
| 3 | 15-0 | `elevator` | sint | LSB 1/100 %, range -100 to 100 |  |
| 4 | 15-0 | `rudder` | sint | LSB 1/100 %, range -100 to 100 |  |
| 5 | 15-0 | `flaps` | sint | LSB 1/100 %, range -100 to 100 |  |
| 6 | 15-0 | `spoilers` | sint | LSB 1/100 %, range -100 to 100 |  |

### FCS_ATTITUDE

**Aircraft attitude** — RT 5 · SA 12 · transmit (terminal to bus) · 4 words · 50 Hz

Replaces: `flight_control_systemOrientationDataRequest`, `flight_control_systemOrientationDataResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `roll_ssm` | ssm | validity of `roll` |  |
| 0 | 13-12 | `pitch_ssm` | ssm | validity of `pitch` |  |
| 0 | 11-10 | `yaw_ssm` | ssm | validity of `yaw` |  |
| 0 | 9-0 | `spare_0` | spare |  |  |
| 1 | 15-0 | `roll` | sint | LSB 180/32768 deg, range -180 to 179.9945 | Right wing down positive |
| 2 | 15-0 | `pitch` | sint | LSB 180/32768 deg, range -90 to 90 | Nose up positive |
| 3 | 15-0 | `yaw` | sint | LSB 180/32768 deg, range -180 to 179.9945 | True heading as -180..+180, nose right positive |

### NAV_SENSORS

**Sensed flight state, for navigation** — RT 7 · SA 13 · receive (bus to terminal) · 12 words · 50 Hz

PLANT_SENSORS relayed to navigation. Airspeed (as ground speed), heading and vertical speed drive the dead-reckoning prediction between GPS fixes, with the rates flightManagementSystem.update_flight_data derived.

Same layout as [`PLANT_SENSORS`](#plant_sensors).

### NAV_GPS

**GPS pseudoranges, for navigation** — RT 7 · SA 14 · receive (bus to terminal) · 25 words · 1 Hz

PLANT_GPS relayed to navigation, which solves the position fix from it (globalPositionSystem.GPSReceiver._wls_solve, then _ecef_to_wgs84).

Same layout as [`PLANT_GPS`](#plant_gps).

### NAV_SOLUTION

**Navigation solution** — RT 7 · SA 15 · transmit (terminal to bus) · 8 words · 10 Hz

The fused position (fuseData.NavDataFusion: one scalar Kalman filter per axis, predicted by dead reckoning every frame and corrected by each GPS fix). position_ssm and altitude_ssm are NO_COMPUTED_DATA until the first fix has been incorporated; after that the estimate is always computed, and gps_valid plus fix_age say how fresh the last fix is. gps_valid is 1 when the last GPS epoch gave a fix (at least 4 satellites, solvable geometry). clock_bias is the receiver clock bias the solver estimated.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `position_ssm` | ssm | validity of `latitude` | Validity of latitude and longitude |
| 0 | 13-12 | `altitude_ssm` | ssm | validity of `altitude` |  |
| 0 | 11 | `gps_valid` | bool |  |  |
| 0 | 10-8 | `spare_0` | spare |  |  |
| 0 | 7-4 | `sats_used` | uint | raw counts | Satellites in the last fix, 0-12 |
| 0 | 3-0 | `spare_1` | spare |  |  |
| 1-2 | 31-0 | `latitude` | sint | LSB 180/2147483648 deg, range -90 to 90 | WGS-84, north positive |
| 3-4 | 31-0 | `longitude` | sint | LSB 180/2147483648 deg, range -180 to 179.9999999 | WGS-84, east positive |
| 5 | 15-0 | `altitude` | sint | LSB 1 ft, range -32768 to 32767 | Height above the WGS-84 ellipsoid, as the GPS reports it |
| 6 | 15-8 | `fix_count` | uint | raw counts | Fixes incorporated since start, saturates at 255 |
| 6 | 7-0 | `fix_age` | uint | raw counts | Seconds since the last fix, saturates at 255; 255 before the first |
| 7 | 15-0 | `clock_bias` | sint | LSB 1/100 us, range -327.68 to 327.67 | Receiver clock bias from the solver |

### FMS_ATTITUDE_IN

**Measured attitude, from the FCS** — RT 12 · SA 12 · receive (bus to terminal) · 4 words · 50 Hz

FCS_ATTITUDE relayed to the FMS: the attitude FMS_FLIGHT_DATA reports, and the "current" values an attitude command leaves alone. flightManagementSystem.update_flight_data copied the FCS attitude the same way.

Same layout as [`FCS_ATTITUDE`](#fcs_attitude).

### FMS_SENSORS

**Sensed flight state, for the FMS** — RT 12 · SA 13 · receive (bus to terminal) · 12 words · 50 Hz

PLANT_SENSORS relayed to the FMS: airspeed, pressure altitude, vertical speed, heading, g and angle of attack for FMS_FLIGHT_DATA and the energy calculations.

Same layout as [`PLANT_SENSORS`](#plant_sensors).

### FMS_NAV_SOLUTION

**Navigation solution, for the FMS** — RT 12 · SA 14 · receive (bus to terminal) · 8 words · 10 Hz

NAV_SOLUTION relayed to the FMS: the position and fused altitude FMS_FLIGHT_DATA reports (navService wrote the fused position into the FMS navigation dict).

Same layout as [`NAV_SOLUTION`](#nav_solution).

### FMS_STATUS

**Flight management status** — RT 12 · SA 15 · transmit (terminal to bus) · 3 words · 5 Hz

Word 0 and the top of word 1 are the terminal-state part every subsystem has. The rest is the FMS flight mode (fmsControl.set_mode) and the tactical systems it sets (fmsControl._update_tactical_systems).

Replaces: `fmsStatusRequest`, `fmsStatusResponse`, `flightManagementSystemStatus`, `fms_modeChangeResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last FMS_MODE_CMD processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last FMS_MODE_CMD was rejected |
| 1 | 14 | `mode_transition` | bool |  | 1 = a commanded mode change is still in progress |
| 1 | 13-12 | `countermeasures` | enum | enum `countermeasures_state` |  |
| 1 | 11-10 | `targeting` | enum | enum `targeting_state` |  |
| 1 | 9-8 | `weapons` | enum | enum `weapons_state` |  |
| 1 | 7 | `stealth_mode` | bool |  | 1 = stealth mode ON |
| 1 | 6-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |
| 2 | 15-12 | `flight_mode` | enum | enum `fms_flight_mode` | The flight mode in force; it selects the flight profile limits |
| 2 | 11-8 | `flight_mode_status` | enum | enum `cmd_status` | Outcome of the last FMS_FLIGHT_MODE_CMD |
| 2 | 7-0 | `flight_mode_cmd_seq` | uint | raw counts | cmd_seq of the last FMS_FLIGHT_MODE_CMD processed |

### FMS_MODE_CMD

**Flight management mode command** — RT 12 · SA 15 · receive (bus to terminal) · 1 words · aperiodic

fmsControl SET_MODE takes only the mode, so the legacy command_parameter word is gone.

Replaces: `fmsCommand`, `flightManagementSystemCommand`, `fms_modeChangeRequest`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### FMS_FLIGHT_MODE_CMD

**Flight mode command** — RT 12 · SA 20 · receive (bus to terminal) · 1 words · aperiodic

fmsControl.set_mode: NORMAL, COMBAT, STEALTH, TRAINING or EMERGENCY. Selects the flight profile limits that attitude commands and maneuvers are held to, and the tactical systems states in FMS_STATUS.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `flight_mode` | enum | enum `fms_flight_mode` |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts |  |

### FMS_ATTITUDE_CMD

**Attitude command** — RT 12 · SA 16 · receive (bus to terminal) · 7 words · aperiodic

Same parameters as fmsControl UPDATE_ATTITUDE. A field is applied only when its present bit is 1.

Replaces: `fms_attitudeUpdateRequest`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15 | `roll_present` | bool |  |  |
| 0 | 14 | `pitch_present` | bool |  |  |
| 0 | 13 | `yaw_present` | bool |  |  |
| 0 | 12 | `roll_rate_present` | bool |  |  |
| 0 | 11 | `pitch_rate_present` | bool |  |  |
| 0 | 10 | `yaw_rate_present` | bool |  |  |
| 0 | 9-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts |  |
| 1 | 15-0 | `roll` | sint | LSB 180/32768 deg, range -180 to 179.9945 |  |
| 2 | 15-0 | `pitch` | sint | LSB 180/32768 deg, range -90 to 90 |  |
| 3 | 15-0 | `yaw` | sint | LSB 180/32768 deg, range -180 to 179.9945 | True |
| 4 | 15-0 | `roll_rate` | sint | LSB 1/64 deg/s, range -500 to 500 |  |
| 5 | 15-0 | `pitch_rate` | sint | LSB 1/64 deg/s, range -500 to 500 |  |
| 6 | 15-0 | `yaw_rate` | sint | LSB 1/64 deg/s, range -500 to 500 |  |

### FMS_ATTITUDE_STATUS

**Attitude command status** — RT 12 · SA 16 · transmit (terminal to bus) · 8 words · 10 Hz

The attitude the FMS holds after envelope limiting (fmsControl.update_attitude): the commanded values, absent ones taken from the measured attitude, each held to the flight profile of the mode in force. A *_limited bit is 1 while that limit is active (the Python's envelope_warnings). The SSMs are NO_COMPUTED_DATA until the first attitude command. The measured attitude is in FMS_FLIGHT_DATA.

Replaces: `fms_attitudeUpdateResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `roll_ssm` | ssm | validity of `roll` |  |
| 0 | 13-12 | `pitch_ssm` | ssm | validity of `pitch` |  |
| 0 | 11-10 | `yaw_ssm` | ssm | validity of `yaw` |  |
| 0 | 9-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts |  |
| 1 | 15 | `bank_limited` | bool |  | Roll held to the profile's max_bank_angle (BANK_ANGLE) |
| 1 | 14 | `pitch_limited` | bool |  | Pitch held to max_pitch_angle (PITCH_ANGLE) |
| 1 | 13 | `roll_rate_limited` | bool |  | Roll rate held to 3 x turn_rate_limit (ROLL_RATE) |
| 1 | 12 | `pitch_rate_limited` | bool |  | Pitch rate held to 2 x turn_rate_limit (PITCH_RATE) |
| 1 | 11 | `yaw_rate_limited` | bool |  | Yaw rate held to turn_rate_limit (YAW_RATE) |
| 1 | 10-4 | `spare_1` | spare |  |  |
| 1 | 3-0 | `status` | enum | enum `cmd_status` | Outcome of the last command |
| 2 | 15-0 | `roll` | sint | LSB 180/32768 deg, range -180 to 179.9945 |  |
| 3 | 15-0 | `pitch` | sint | LSB 180/32768 deg, range -90 to 90 |  |
| 4 | 15-0 | `yaw` | sint | LSB 180/32768 deg, range -180 to 179.9945 | True |
| 5 | 15-0 | `roll_rate` | sint | LSB 1/64 deg/s, range -500 to 500 |  |
| 6 | 15-0 | `pitch_rate` | sint | LSB 1/64 deg/s, range -500 to 500 |  |
| 7 | 15-0 | `yaw_rate` | sint | LSB 1/64 deg/s, range -500 to 500 |  |

### FMS_NAV_UPDATE_CMD

**Autopilot target command** — RT 12 · SA 17 · receive (bus to terminal) · 5 words · aperiodic

The legacy update_type and parameter_count had no meaning in the code. This carries the targets of fmsControl SET_AUTOPILOT; a target is applied only when its present bit is 1.

Replaces: `fms_navigationUpdateRequest`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15 | `altitude_present` | bool |  |  |
| 0 | 14 | `airspeed_present` | bool |  |  |
| 0 | 13 | `heading_present` | bool |  |  |
| 0 | 12 | `vertical_speed_present` | bool |  |  |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts |  |
| 1 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 60000 | Limits from fmsControl.set_autopilot_targets |
| 2 | 15-0 | `airspeed` | uint | LSB 1/10 kt, range 0 to 1500 |  |
| 3 | 15-0 | `heading` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True |
| 4 | 15-0 | `vertical_speed` | sint | LSB 1 ft/min, range -30000 to 30000 | Climb positive |

### FMS_NAV_UPDATE_STATUS

**Autopilot target status** — RT 12 · SA 17 · transmit (terminal to bus) · 6 words · 5 Hz

The targets in force after fmsControl.set_autopilot_targets limited them: altitude 0-60000 ft, airspeed 100-1000 kt, heading wrapped to 0-360, vertical speed -8000 .. 6000 ft/min. Before any command they are the Python's initial targets: 30000 ft, 450 kt, 0 deg, 0 ft/min.

Replaces: `fms_navigationUpdateResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts |  |
| 1 | 15-4 | `spare_1` | spare |  |  |
| 1 | 3-0 | `status` | enum | enum `cmd_status` | Outcome of the last command |
| 2 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 60000 |  |
| 3 | 15-0 | `airspeed` | uint | LSB 1/10 kt, range 0 to 1500 |  |
| 4 | 15-0 | `heading` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True |
| 5 | 15-0 | `vertical_speed` | sint | LSB 1 ft/min, range -30000 to 30000 | Climb positive |

### FMS_MANEUVER_CMD

**Maneuver command** — RT 12 · SA 18 · receive (bus to terminal) · 3 words · aperiodic

Same maneuvers as fmsControl.execute_maneuver(). DIVE is the only one that takes a parameter.

Replaces: `fms_maneuverRequest`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts |  |
| 1 | 15-4 | `spare_1` | spare |  |  |
| 1 | 3-0 | `maneuver` | enum | enum `maneuver` |  |
| 2 | 15-0 | `dive_angle` | sint | LSB 180/32768 deg, range -90 to 0 | DIVE only; 0 means the FMS default of -30 deg |

### FMS_MANEUVER_STATUS

**Maneuver status** — RT 12 · SA 18 · transmit (terminal to bus) · 2 words · 5 Hz

Replaces: `fms_maneuverResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts |  |
| 1 | 15-8 | `spare_1` | spare |  |  |
| 1 | 7-4 | `maneuver` | enum | enum `maneuver` | The last maneuver commanded |
| 1 | 3-0 | `status` | enum | enum `cmd_status` | Outcome of the last command |

### FMS_FLIGHT_DATA

**FMS flight data** — RT 12 · SA 19 · transmit (terminal to bus) · 21 words · 10 Hz

What flightManagementSystem.get_flight_data() gave the displays: attitude from the FCS (FMS_ATTITUDE_IN), air data from the sensors (FMS_SENSORS), position and fused altitude from navigation (FMS_NAV_SOLUTION), and the values the Python derived - mach = airspeed / (661.4788 x 0.98); energy_state = min(100, (0.5 x 15000 x v^2 + 15000 x 9.81 x h) / 1e10 x 100) with v in m/s from airspeed and h in m from nav_altitude. ground_speed is the airspeed, since the plant has no wind. track and sideslip were never computed by the Python and are sent NO_COMPUTED_DATA. Replaces the FMS_DATA block of v1.

Replaces: `fmsDataRequest`, `fmsDataResponse`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `roll_ssm` | ssm | validity of `roll` |  |
| 0 | 13-12 | `pitch_ssm` | ssm | validity of `pitch` |  |
| 0 | 11-10 | `heading_ssm` | ssm | validity of `heading` |  |
| 0 | 9-8 | `airspeed_ssm` | ssm | validity of `airspeed` |  |
| 0 | 7-6 | `altitude_ssm` | ssm | validity of `altitude` |  |
| 0 | 5-4 | `vertical_speed_ssm` | ssm | validity of `vertical_speed` |  |
| 0 | 3-2 | `g_ssm` | ssm | validity of `g_force` |  |
| 0 | 1-0 | `aoa_ssm` | ssm | validity of `angle_of_attack` |  |
| 1 | 15-14 | `position_ssm` | ssm | validity of `latitude` | Validity of latitude and longitude |
| 1 | 13-12 | `nav_altitude_ssm` | ssm | validity of `nav_altitude` |  |
| 1 | 11-10 | `mach_ssm` | ssm | validity of `mach` |  |
| 1 | 9-8 | `ground_speed_ssm` | ssm | validity of `ground_speed` |  |
| 1 | 7-6 | `track_ssm` | ssm | validity of `track` |  |
| 1 | 5-4 | `sideslip_ssm` | ssm | validity of `sideslip` |  |
| 1 | 3-2 | `energy_ssm` | ssm | validity of `energy_state` |  |
| 1 | 1-0 | `spare_1` | spare |  |  |
| 2 | 15-12 | `flight_mode` | enum | enum `fms_flight_mode` | The legacy status.mode the PFD showed |
| 2 | 11-0 | `spare_2` | spare |  |  |
| 3-4 | 31-0 | `latitude` | sint | LSB 180/2147483648 deg, range -90 to 90 | WGS-84, north positive |
| 5-6 | 31-0 | `longitude` | sint | LSB 180/2147483648 deg, range -180 to 179.9999999 | WGS-84, east positive |
| 7 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 65535 | Pressure altitude from the sensors |
| 8 | 15-0 | `nav_altitude` | sint | LSB 1 ft, range -32768 to 32767 | The navigation solution's fused altitude: what the Python's navigation.altitude was |
| 9 | 15-0 | `airspeed` | uint | LSB 1/10 kt, range 0 to 1500 | Indicated airspeed |
| 10 | 15-0 | `mach` | uint | LSB 1/1000 mach, range 0 to 5 |  |
| 11 | 15-0 | `ground_speed` | uint | LSB 1/10 kt, range 0 to 1500 |  |
| 12 | 15-0 | `vertical_speed` | sint | LSB 1 ft/min, range -30000 to 30000 | Climb positive |
| 13 | 15-0 | `heading` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True |
| 14 | 15-0 | `track` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True ground track |
| 15 | 15-0 | `roll` | sint | LSB 180/32768 deg, range -180 to 179.9945 | Right wing down positive |
| 16 | 15-0 | `pitch` | sint | LSB 180/32768 deg, range -90 to 90 | Nose up positive |
| 17 | 15-0 | `g_force` | sint | LSB 1/1000 g, range -20 to 20 |  |
| 18 | 15-0 | `angle_of_attack` | sint | LSB 180/32768 deg, range -90 to 90 |  |
| 19 | 15-0 | `sideslip` | sint | LSB 180/32768 deg, range -90 to 90 |  |
| 20 | 15-0 | `energy_state` | uint | LSB 1/100 %, range 0 to 100 | The Python's tactical energy_state |

### FMS_CONTROL_CMD

**Thrust and landing gear command** — RT 12 · SA 21 · receive (bus to terminal) · 2 words · aperiodic

fmsControl SET_THRUST and SET_LANDING_GEAR, which message_schemas.py never carried. Each part is applied only when its present bit is 1. Gear accepts UP and DOWN only.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15 | `thrust_present` | bool |  |  |
| 0 | 14 | `gear_present` | bool |  |  |
| 0 | 13-12 | `gear` | enum | enum `gear_position` | UP or DOWN; anything else is REJECTED_INVALID |
| 0 | 11-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts |  |
| 1 | 15-0 | `thrust` | uint | LSB 1/10000, range 0 to 1 | Requested thrust, 0-1 (fmsControl.set_thrust thrust_level) |

### FMS_CONTROL_STATUS

**Thrust and landing gear status** — RT 12 · SA 21 · transmit (terminal to bus) · 3 words · 5 Hz

fmsControl.set_thrust holds thrust to 0.1 .. 0.9 (1.0 with the afterburner lit), sets throttle_position = int(thrust x 100), lights the afterburner above 0.9 and puts it out below 0.85. Before any command: thrust 0.5, throttle 50, afterburner off, gear UP.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-8 | `spare_0` | spare |  |  |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts |  |
| 1 | 15 | `afterburner` | bool |  |  |
| 1 | 14-13 | `gear` | enum | enum `gear_position` | The commanded gear position |
| 1 | 12-6 | `throttle_position` | uint | raw counts | Percent, 0-100: int(thrust x 100) |
| 1 | 5-4 | `spare_1` | spare |  |  |
| 1 | 3-0 | `status` | enum | enum `cmd_status` | Outcome of the last command |
| 2 | 15-0 | `thrust` | uint | LSB 1/10000, range 0 to 1 | Thrust in force, 0-1 |

### FMS_ENERGY

**Energy maneuverability** — RT 12 · SA 22 · transmit (terminal to bus) · 6 words · 1 Hz

fmsControl.calculate_energy_maneuverability(), from nav_altitude, airspeed, angle of attack and the flight profile's engine_power and max_g_force: specific excess power Ps = v (T - D) / W with T = engine_power x 150000 N, D = 0.5 x 1.225 e^(-h/10000) v^2 (0.02 + \|aoa\|/100) x 30, W = 15000 x 9.81; sustained turn rate = deg(g sqrt((T/W)^2 - 1) / v) when T/W is above 1, else 0; instantaneous turn rate = deg(g sqrt(max_g^2 - 1) / v); climb = Ps x 196.85 ft/min; energy_advantage = 10 Ps. v is at least 1 m/s in the turn rates.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `ssm` | ssm | validity of `specific_excess_power` | Validity of all five values |
| 0 | 13-0 | `spare_0` | spare |  |  |
| 1 | 15-0 | `specific_excess_power` | sint | LSB 1/50 m/s, range -655.36 to 655.34 |  |
| 2 | 15-0 | `max_sustained_turn_rate` | uint | LSB 1/10 deg/s, range 0 to 6553.5 |  |
| 3 | 15-0 | `max_instantaneous_turn_rate` | uint | LSB 1/10 deg/s, range 0 to 6553.5 |  |
| 4 | 15-0 | `max_climb_rate` | sint | LSB 4 ft/min, range -131072 to 131068 |  |
| 5 | 15-0 | `energy_advantage` | sint | LSB 1/5, range -6553.6 to 6553.4 | 10 x specific_excess_power |

### DSP_PFD_STATUS

**Primary flight display status** — RT 11 · SA 11 · transmit (terminal to bus) · 2 words · 1 Hz

Replaces: `displayStatusRequest`, `displayStatusResponse`, `display_mode_response`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `display_mode` | Current lighting mode |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### DSP_PFD_MODE_CMD

**Primary flight display mode command** — RT 11 · SA 11 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `displayCommand`, `display_mode_request`, `display_mode_change`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `display_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### DSP_MFD_STATUS

**Multi-function display status** — RT 13 · SA 12 · transmit (terminal to bus) · 2 words · 1 Hz

Replaces: `displayStatusRequest`, `displayStatusResponse`, `display_mode_response`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `display_mode` | Current lighting mode |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### DSP_MFD_MODE_CMD

**Multi-function display mode command** — RT 13 · SA 12 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `displayCommand`, `display_mode_request`, `display_mode_change`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `display_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### DSP_EICAS_STATUS

**EICAS status** — RT 14 · SA 13 · transmit (terminal to bus) · 2 words · 1 Hz

Replaces: `displayStatusRequest`, `displayStatusResponse`, `display_mode_response`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `display_mode` | Current lighting mode |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### DSP_EICAS_MODE_CMD

**EICAS mode command** — RT 14 · SA 13 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `displayCommand`, `display_mode_request`, `display_mode_change`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `display_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### DSP_RADAR_STATUS

**Weather radar display status** — RT 15 · SA 14 · transmit (terminal to bus) · 2 words · 1 Hz

Replaces: `displayStatusRequest`, `displayStatusResponse`, `display_mode_response`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `display_mode` | Current lighting mode |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### DSP_RADAR_MODE_CMD

**Weather radar display mode command** — RT 15 · SA 14 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `displayCommand`, `display_mode_request`, `display_mode_change`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `display_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### DSP_TSD_STATUS

**Tactical situation display status** — RT 16 · SA 15 · transmit (terminal to bus) · 2 words · 1 Hz

Replaces: `displayStatusRequest`, `displayStatusResponse`, `display_mode_response`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `display_mode` | Current lighting mode |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### DSP_TSD_MODE_CMD

**Tactical situation display mode command** — RT 16 · SA 15 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `displayCommand`, `display_mode_request`, `display_mode_change`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `display_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### DSP_SMS_STATUS

**Stores management display status** — RT 17 · SA 16 · transmit (terminal to bus) · 2 words · 1 Hz

Replaces: `displayStatusRequest`, `displayStatusResponse`, `display_mode_response`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `display_mode` | Current lighting mode |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### DSP_SMS_MODE_CMD

**Stores management display mode command** — RT 17 · SA 16 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `displayCommand`, `display_mode_request`, `display_mode_change`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `display_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### DSP_HUD_STATUS

**Head-up display status** — RT 18 · SA 17 · transmit (terminal to bus) · 2 words · 1 Hz

Replaces: `displayStatusRequest`, `displayStatusResponse`, `display_mode_response`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `mode` | enum | enum `display_mode` | Current lighting mode |
| 0 | 7-0 | `last_cmd_seq` | uint | raw counts | cmd_seq of the last command processed (see Command sequence rule) |
| 1 | 15 | `cmd_rejected` | bool |  | 1 = the last command was rejected |
| 1 | 14-1 | `spare_1` | spare |  |  |
| 1 | 0 | `healthy` | bool |  | 1 = healthy (legacy health_status, bit 0) |

### DSP_HUD_MODE_CMD

**Head-up display mode command** — RT 18 · SA 17 · receive (bus to terminal) · 1 words · aperiodic

Replaces: `displayCommand`, `display_mode_request`, `display_mode_change`

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `requested_state` | enum | enum `terminal_state` |  |
| 0 | 11-8 | `requested_mode` | enum | enum `display_mode` |  |
| 0 | 7-0 | `cmd_seq` | uint | raw counts | Incremented by the sender for each new command |

### MFD_RDR_WX_CMD

**MFD command to the weather radar** — RT 13 · SA 20 · transmit (terminal to bus) · 3 words · 10 Hz · **PROPOSED**

Copied by the BC into RT 9 SA 1. Confirm against the display inventory (v2).

Same layout as [`RDR_WX_MODE_CMD`](#rdr_wx_mode_cmd).

### MFD_RDR_TFR_CMD

**MFD command to the terrain-following radar** — RT 13 · SA 21 · transmit (terminal to bus) · 2 words · 10 Hz · **PROPOSED**

Copied by the BC into RT 9 SA 2. Confirm against the display inventory (v2).

Same layout as [`RDR_TFR_MODE_CMD`](#rdr_tfr_mode_cmd).

### MFD_RDR_SAR_CMD

**MFD command to the synthetic aperture radar** — RT 13 · SA 22 · transmit (terminal to bus) · 2 words · 10 Hz · **PROPOSED**

Copied by the BC into RT 9 SA 3. Confirm against the display inventory (v2).

Same layout as [`RDR_SAR_MODE_CMD`](#rdr_sar_mode_cmd).

### MFD_RDR_TGT_CMD

**MFD command to the targeting radar** — RT 13 · SA 23 · transmit (terminal to bus) · 2 words · 10 Hz · **PROPOSED**

Copied by the BC into RT 9 SA 4. Confirm against the display inventory (v2).

Same layout as [`RDR_TGT_MODE_CMD`](#rdr_tgt_mode_cmd).

### MFD_RDR_AEWC_CMD

**MFD command to the aewc radar** — RT 13 · SA 24 · transmit (terminal to bus) · 4 words · 10 Hz · **PROPOSED**

Copied by the BC into RT 9 SA 5. Confirm against the display inventory (v2).

Same layout as [`RDR_AEWC_MODE_CMD`](#rdr_aewc_mode_cmd).

### PLANT_SENSORS

**Sensed flight state from the plant** — RT 27 · SA 1 · transmit (terminal to bus) · 12 words · 50 Hz

What the aircraft's sensors would measure, published by the simulated aircraft. Ranges follow flightControlComputer.py (altitude 0-60000 ft, speed 100-1000 kt, pitch +-45, roll +-60, AoA 0-25, g 0.5-9) with headroom. Each quantity carries its own SSM so a sensor can fail alone.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `altitude_ssm` | ssm | validity of `altitude` |  |
| 0 | 13-12 | `airspeed_ssm` | ssm | validity of `airspeed` |  |
| 0 | 11-10 | `pitch_ssm` | ssm | validity of `pitch` |  |
| 0 | 9-8 | `roll_ssm` | ssm | validity of `roll` |  |
| 0 | 7-6 | `heading_ssm` | ssm | validity of `heading` |  |
| 0 | 5-4 | `vertical_speed_ssm` | ssm | validity of `vertical_speed` |  |
| 0 | 3-2 | `aoa_ssm` | ssm | validity of `angle_of_attack` |  |
| 0 | 1-0 | `g_ssm` | ssm | validity of `g_force` |  |
| 1 | 15-14 | `clearance_ssm` | ssm | validity of `terrain_clearance` |  |
| 1 | 13-12 | `flaps_ssm` | ssm | validity of `flaps` |  |
| 1 | 11-10 | `gear` | enum | enum `gear_position` |  |
| 1 | 9-0 | `spare_1` | spare |  |  |
| 2 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 65535 | Pressure altitude |
| 3 | 15-0 | `airspeed` | uint | LSB 1/10 kt, range 0 to 1500 | Indicated airspeed |
| 4 | 15-0 | `vertical_speed` | sint | LSB 1 ft/min, range -30000 to 30000 | Climb positive |
| 5 | 15-0 | `pitch` | sint | LSB 180/32768 deg, range -90 to 90 | Nose up positive |
| 6 | 15-0 | `roll` | sint | LSB 180/32768 deg, range -180 to 179.9945 | Right wing down positive |
| 7 | 15-0 | `heading` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True |
| 8 | 15-0 | `angle_of_attack` | sint | LSB 180/32768 deg, range -90 to 90 |  |
| 9 | 15-0 | `g_force` | sint | LSB 1/1000 g, range -20 to 20 | Normal load factor, 1.0 in level flight |
| 10 | 15-0 | `flaps` | uint | LSB 1/100 deg, range 0 to 30 | fmsControl flaps range |
| 11 | 15-0 | `terrain_clearance` | uint | LSB 1 ft, range 0 to 65535 | Height above terrain. From the plant until the radar terminal publishes it |

### PLANT_GPS

**GPS pseudoranges from the plant** — RT 27 · SA 2 · transmit (terminal to bus) · 25 words · 1 Hz

What the simulated GPS receiver measures each epoch: for every satellite of the 12-satellite constellation of globalPositionSystem._build_constellation() that is at least 5 degrees above the horizon, pseudorange = geometric range + c x (receiver clock bias - satellite clock bias) + measurement noise (3 m, 1 sigma). The noise is drawn once per epoch here, in the plant; the solver in the navigation terminal is deterministic. epoch increments with each new set; a pseudorange whose satellite bit is 0 in visible_mask is not valid.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-12 | `epoch` | uint | raw counts | Increments (mod 16) with every new measurement set |
| 0 | 11-0 | `visible_mask` | uint | raw counts | Bit n-1 = satellite n (1-12) has a valid pseudorange |
| 1-2 | 31-0 | `pseudorange_1` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 3-4 | 31-0 | `pseudorange_2` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 5-6 | 31-0 | `pseudorange_3` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 7-8 | 31-0 | `pseudorange_4` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 9-10 | 31-0 | `pseudorange_5` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 11-12 | 31-0 | `pseudorange_6` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 13-14 | 31-0 | `pseudorange_7` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 15-16 | 31-0 | `pseudorange_8` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 17-18 | 31-0 | `pseudorange_9` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 19-20 | 31-0 | `pseudorange_10` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 21-22 | 31-0 | `pseudorange_11` | sint | LSB 1/16 m, range 0 to 100000000 |  |
| 23-24 | 31-0 | `pseudorange_12` | sint | LSB 1/16 m, range 0 to 100000000 |  |

### PLANT_WEATHER

**Weather truth from the plant** — RT 27 · SA 3 · transmit (terminal to bus) · 32 words · 1 Hz

The simulated weather the radar's sensor model renders: the storm cells and the stratiform area of weather_state_manager.py, which the plant moves with the wind and ages through their lifecycles as that module did, published once a second. The radar never sees this as weather; it renders a reflectivity volume from it, as reflectivity_simulator.py did, and processes that. Cells beyond cell_count and an absent area are zero. A cell's vertical profile is one of the two the Python used: entry_profile 0 is the profile of the initial cells (1.0, 0.9, 0.8, 0.7 for the lowest four elevations, then 0.2), 1 the profile of cells that entered at the edge of the picture (1 - 0.15 per elevation, floor 0.2).

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-8 | `weather_seq` | uint | raw counts | Increments (mod 256) with every publication |
| 0 | 7-4 | `cell_count` | uint | raw counts | Storm cells present, 0-8 |
| 0 | 3-0 | `area_count` | uint | raw counts | Stratiform areas present, 0-1 |
| 1 | 15-0 | `cell_1_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 1 centre, true azimuth |
| 2 | 15-0 | `cell_1_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 1 centre, range |
| 3 | 15-9 | `cell_1_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 1 radius |
| 3 | 8 | `cell_1_entry_profile` | bool |  | Cell 1 vertical profile: 0 initial, 1 entered at the edge |
| 3 | 7-0 | `cell_1_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 1 peak reflectivity |
| 4 | 15-0 | `cell_2_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 2 centre, true azimuth |
| 5 | 15-0 | `cell_2_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 2 centre, range |
| 6 | 15-9 | `cell_2_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 2 radius |
| 6 | 8 | `cell_2_entry_profile` | bool |  | Cell 2 vertical profile: 0 initial, 1 entered at the edge |
| 6 | 7-0 | `cell_2_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 2 peak reflectivity |
| 7 | 15-0 | `cell_3_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 3 centre, true azimuth |
| 8 | 15-0 | `cell_3_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 3 centre, range |
| 9 | 15-9 | `cell_3_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 3 radius |
| 9 | 8 | `cell_3_entry_profile` | bool |  | Cell 3 vertical profile: 0 initial, 1 entered at the edge |
| 9 | 7-0 | `cell_3_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 3 peak reflectivity |
| 10 | 15-0 | `cell_4_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 4 centre, true azimuth |
| 11 | 15-0 | `cell_4_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 4 centre, range |
| 12 | 15-9 | `cell_4_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 4 radius |
| 12 | 8 | `cell_4_entry_profile` | bool |  | Cell 4 vertical profile: 0 initial, 1 entered at the edge |
| 12 | 7-0 | `cell_4_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 4 peak reflectivity |
| 13 | 15-0 | `cell_5_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 5 centre, true azimuth |
| 14 | 15-0 | `cell_5_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 5 centre, range |
| 15 | 15-9 | `cell_5_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 5 radius |
| 15 | 8 | `cell_5_entry_profile` | bool |  | Cell 5 vertical profile: 0 initial, 1 entered at the edge |
| 15 | 7-0 | `cell_5_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 5 peak reflectivity |
| 16 | 15-0 | `cell_6_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 6 centre, true azimuth |
| 17 | 15-0 | `cell_6_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 6 centre, range |
| 18 | 15-9 | `cell_6_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 6 radius |
| 18 | 8 | `cell_6_entry_profile` | bool |  | Cell 6 vertical profile: 0 initial, 1 entered at the edge |
| 18 | 7-0 | `cell_6_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 6 peak reflectivity |
| 19 | 15-0 | `cell_7_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 7 centre, true azimuth |
| 20 | 15-0 | `cell_7_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 7 centre, range |
| 21 | 15-9 | `cell_7_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 7 radius |
| 21 | 8 | `cell_7_entry_profile` | bool |  | Cell 7 vertical profile: 0 initial, 1 entered at the edge |
| 21 | 7-0 | `cell_7_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 7 peak reflectivity |
| 22 | 15-0 | `cell_8_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Cell 8 centre, true azimuth |
| 23 | 15-0 | `cell_8_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Cell 8 centre, range |
| 24 | 15-9 | `cell_8_size` | uint | LSB 1/4 km, range 0 to 31.75 | Cell 8 radius |
| 24 | 8 | `cell_8_entry_profile` | bool |  | Cell 8 vertical profile: 0 initial, 1 entered at the edge |
| 24 | 7-0 | `cell_8_intensity` | uint | LSB 1/2 dBZ, range 0 to 127.5 | Cell 8 peak reflectivity |
| 25 | 15-0 | `area_azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Area centre, true azimuth |
| 26 | 15-0 | `area_range` | uint | LSB 1/64 km, range 0 to 1023.98 | Area centre, range |
| 27 | 15-0 | `area_width` | uint | LSB 1/128 deg, range 0 to 511.99 | Area width in azimuth |
| 28 | 15-0 | `area_depth` | uint | LSB 1/64 km, range 0 to 1023.98 | Area depth in range |
| 29 | 15-0 | `area_intensity` | uint | LSB 1/128 dBZ, range 0 to 511.99 | Area reflectivity at its centre |
| 30 | 15-0 | `spare_30` | spare |  |  |
| 31 | 15-0 | `spare_31` | spare |  |  |

### FCS_SENSORS

**Sensed flight state as received by the flight control computer** — RT 5 · SA 13 · receive (bus to terminal) · 12 words · 50 Hz

Relayed by the bus controller from PLANT_SENSORS in the same minor frame (relays section). The FCS computes FCS_ATTITUDE, GCAS and the performance envelope from it.

Same layout as [`PLANT_SENSORS`](#plant_sensors).

### FCS_GCAS_ALERTS

**Ground collision avoidance alerts** — RT 5 · SA 14 · transmit (terminal to bus) · 4 words · 5 Hz

groundCollisionAvoidanceSys.py, evaluated every frame: PULLUP when vertical speed is at or below -4000 ft/min and altitude is below 2000 ft; otherwise SINK_RATE when vertical speed is at or below -2000 ft/min; TERRAIN when terrain clearance is below 500 ft; ALT_LOW when altitude is below 200 ft and TERRAIN is not set. PULLUP, SINK_RATE and TERRAIN are severity 1 (WARNING); ALT_LOW is severity 2 (CAUTION). The values that drove the decision ride along so a display can show them without another message.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-14 | `status` | enum | enum `gcas_status` |  |
| 0 | 13 | `pullup` | bool |  |  |
| 0 | 12 | `sink_rate` | bool |  |  |
| 0 | 11 | `terrain` | bool |  |  |
| 0 | 10 | `alt_low` | bool |  |  |
| 0 | 9-8 | `vertical_speed_ssm` | ssm | validity of `vertical_speed` |  |
| 0 | 7-6 | `altitude_ssm` | ssm | validity of `altitude` |  |
| 0 | 5-4 | `clearance_ssm` | ssm | validity of `terrain_clearance` |  |
| 0 | 3-0 | `spare_0` | spare |  |  |
| 1 | 15-0 | `vertical_speed` | sint | LSB 1 ft/min, range -30000 to 30000 |  |
| 2 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 65535 |  |
| 3 | 15-0 | `terrain_clearance` | uint | LSB 1 ft, range 0 to 65535 |  |

### FCS_PERF_STATUS

**Flight envelope monitoring** — RT 5 · SA 15 · transmit (terminal to bus) · 4 words · 1 Hz

performaneMonitoring.py: an exceedance bit is set while the magnitude of the parameter is above its limit - g 9.0, angle of attack 25 deg, roll 60 deg, pitch 45 deg, vertical speed 6000 ft/min. Fuel efficiency is airspeed / (fuel flow / 60) in nm per kg and is NO_COMPUTED_DATA until the engine terminal publishes fuel flow.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15 | `exceed_g` | bool |  |  |
| 0 | 14 | `exceed_aoa` | bool |  |  |
| 0 | 13 | `exceed_roll` | bool |  |  |
| 0 | 12 | `exceed_pitch` | bool |  |  |
| 0 | 11 | `exceed_vertical_speed` | bool |  |  |
| 0 | 10-9 | `g_ssm` | ssm | validity of `g_force` |  |
| 0 | 8-7 | `aoa_ssm` | ssm | validity of `angle_of_attack` |  |
| 0 | 6-5 | `fuel_ssm` | ssm | validity of `fuel_efficiency` |  |
| 0 | 4-0 | `spare_0` | spare |  |  |
| 1 | 15-0 | `g_force` | sint | LSB 1/1000 g, range -20 to 20 |  |
| 2 | 15-0 | `angle_of_attack` | sint | LSB 180/32768 deg, range -90 to 90 |  |
| 3 | 15-0 | `fuel_efficiency` | uint | LSB 1/1000 nm/kg, range 0 to 65.535 |  |

## Block payloads

### RDR_WX_PRODUCTS

**Weather radar products** — RT 9 · descriptor SA 6 · segment SA 7 · 50 Hz · payload at most 548 words

Everything one scan produced, published as one block when the scan completes so a consumer always draws a consistent picture (README section 8). Which sections are filled depends on the mode: SURVEILLANCE fills vil, precip and cells; MAPPING and PRECIPITATION fill precip and areas; WINDSHEAR fills windshear; TURBULENCE fills turbulence. Positions are polar from the aircraft: true azimuth and range. The record limits apply after the selection described in the README (the Python's own limits for the VIL and precipitation points; new for the rest); the counts before the limits are in RDR_WX_SCAN. The segment subaddress is polled every frame, so an 18-segment block arrives in 0.36 s, well inside the 1.2 s scan.

Replaces: `weather_radarVILResponse`, `DISPLAY_VIL_DATA`, `weather_radarData`, `radarDataRequest`, `radarDataResponse`, `weather_radarPrecipitationResponse`

**Header (2 words)**, followed by one count word per section in the order below.

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-8 | `scan_id` | uint | raw counts | Scan these products come from (RDR_WX_SCAN.scan_id) |
| 0 | 7-4 | `mode` | enum | enum `weather_mode` | Mode of that scan; NONE with every section empty when the radar is not scanning |
| 0 | 3 | `microburst` | bool |  | 1 = a microburst was detected (WINDSHEAR mode) |
| 0 | 2-0 | `max_turbulence` | enum | enum `turbulence_category` | Strongest turbulence category (TURBULENCE mode) |
| 1 | 15-0 | `max_reflectivity` | sint | LSB 1/128 dBZ, range -20 to 65 | Strongest return in the volume |

**Section `vil` — Vertically integrated liquid**: up to 20 records of 4 words. vil_data_generator_sync: VIL per column, the significant columns of the 8x-downsampled field, at most 20 chosen for continuity with the previous scan and then by cluster intensity

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-0 | `azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True azimuth of the column |
| 1 | 15-0 | `range` | uint | LSB 1/64 km, range 0 to 1023.98 | Range of the column |
| 2 | 15-0 | `value` | uint | LSB 1/4 kg/m2, range 0 to 16383.75 | VIL as the Python computed it: 3.44e-3 x Z^(4/7) x layer thickness, summed over the elevations |
| 3 | 15-8 | `intensity` | uint | LSB 1/200, range 0 to 1.275 | min(1, value / 65) |
| 3 | 7-4 | `layer_count` | uint | raw counts | min(15, int(intensity x 15) + 1) |
| 3 | 3 | `show_values` | bool |  | 1 = intensity above 0.3: the display labels the point |
| 3 | 2-0 | `spare_3` | spare |  |  |

**Section `precip` — Precipitation points**: up to 15 records of 4 words. precipitation_data_generator_sync: the Marshall-Palmer rate of each column's strongest return, the significant columns of the 8x-downsampled field, at most 15 chosen as the VIL points are

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-0 | `azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True azimuth of the column |
| 1 | 15-0 | `range` | uint | LSB 1/64 km, range 0 to 1023.98 | Range of the column |
| 2 | 15-0 | `rate` | uint | LSB 1/10 mm/h, range 0 to 6553.5 | (Z / 200)^(1 / 1.6) |
| 3 | 15-8 | `intensity` | uint | LSB 1/200, range 0 to 1.275 | min(1, rate / 50) |
| 3 | 7-5 | `type` | enum | enum `precip_type` | By rate: HAIL above 50, MIXED above 25, RAIN above 10, else SNOW three times in ten, RAIN otherwise |
| 3 | 4 | `show_values` | bool |  | 1 = intensity above 0.3 |
| 3 | 3-0 | `spare_3` | spare |  |  |

**Section `cells` — Storm cells**: up to 12 records of 10 words. stormCellTracking.StormCellTracker: connected regions above 30 dBZ in the composite (maximum over elevation) field, tracked from scan to scan by proximity (5 bins); the 12 largest by size, then reflectivity, are reported

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-0 | `cell_id` | uint | raw counts | Tracker id, kept while the cell is followed |
| 1 | 15-0 | `azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Centroid, true azimuth |
| 2 | 15-0 | `range` | uint | LSB 1/64 km, range 0 to 1023.98 | Centroid range |
| 3 | 15-0 | `reflectivity` | uint | LSB 1/128 dBZ, range 0 to 511.99 | Mean over the region |
| 4 | 15-0 | `size` | uint | LSB 1/2 nm, range 0 to 32767.5 | Region size, half a nautical mile per bin as the Python counted it |
| 5 | 15-0 | `intensity` | uint | LSB 1/10000, range 0 to 1 | (reflectivity - 30) / 35, limited to 0..1 |
| 6 | 15-0 | `azimuth_rate` | sint | LSB 1/64 deg/s, range -512 to 511.98 | Centroid motion since the previous scan |
| 7 | 15-0 | `range_rate` | sint | LSB 1/64 km/s, range -512 to 511.98 | Centroid motion since the previous scan |
| 8 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 65535 | The Python's cell-top estimate: size (as a range proxy) x tan(elevation) |
| 9 | 15-0 | `vertical_development` | sint | LSB 1/16 ft/min, range -2048 to 2047.9 | Change of that estimate per scan interval |

**Section `areas` — Precipitation areas**: up to 12 records of 5 words. precipitation_analysis.PrecipitationAnalyzer: connected regions above 30 dBZ, with the Marshall-Palmer rate of their mean reflectivity and a type from it; the 12 strongest by reflectivity are reported

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-0 | `azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | Centroid, true azimuth |
| 1 | 15-0 | `range` | uint | LSB 1/64 km, range 0 to 1023.98 | Centroid range |
| 2 | 15-0 | `reflectivity` | uint | LSB 1/128 dBZ, range 0 to 511.99 | Mean over the region |
| 3 | 15-0 | `rate` | uint | LSB 1/10 mm/h, range 0 to 6553.5 | (Z / 200)^(1 / 1.6) of the mean |
| 4 | 15-8 | `intensity` | uint | LSB 1/200, range 0 to 1.275 | (reflectivity - 30) / 35, limited to 0..1 |
| 4 | 7-5 | `type` | enum | enum `precip_type` | HAIL above 55 dBZ, HEAVY_RAIN above 45, else RAIN |
| 4 | 4-0 | `spare_4` | spare |  |  |

**Section `windshear` — Wind shear events**: up to 20 records of 5 words. weather_processor.WindShearProcessor on the lowest elevation: shear proxy = 0.8 x \|azimuthal reflectivity gradient\| in kt/nm, an event where that is 15 or more and reflectivity 20 dBZ or more, one per 5x5-bin cluster, in order of severity; the first 20 are reported

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-0 | `azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True azimuth of the bin |
| 1 | 15-0 | `range` | uint | LSB 1/64 km, range 0 to 1023.98 | Range of the bin |
| 2 | 15-0 | `shear` | uint | LSB 1/16 kt/nm, range 0 to 4095.9 |  |
| 3 | 15-14 | `severity` | enum | enum `shear_severity` |  |
| 3 | 13 | `microburst` | bool |  | 1 = divergence above 0.08 |
| 3 | 12-0 | `divergence` | sint | LSB 1/512, range -8 to 7.998 | Range gradient / 20; positive is outflow |
| 4 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 65535 | Beam height at the bin: range x tan(lowest elevation) |

**Section `turbulence` — Turbulence cells**: up to 20 records of 6 words. weather_processor.TurbulenceProcessor on the lowest elevation: spectrum width proxy = local 3x3 standard deviation of reflectivity, EDR proxy = 0.3 x width^(2/3) / range_m^(1/3), a cell where EDR is 0.1 or more, reflectivity 15 dBZ or more and width at least 1, one per 4x4-bin cluster, strongest first; the first 20 are reported

| Word | Bits | Field | Type | Scaling | Notes |
|---|---|---|---|---|---|
| 0 | 15-0 | `azimuth` | uint | LSB 360/65536 deg, range 0 to 359.9945 | True azimuth of the bin |
| 1 | 15-0 | `range` | uint | LSB 1/64 km, range 0 to 1023.98 | Range of the bin |
| 2 | 15-0 | `intensity` | uint | LSB 1/10000, range 0 to 1 | min(1, EDR / 0.6) |
| 3 | 15-0 | `spectrum_width` | uint | LSB 1/64 m/s, range 0 to 1023.98 |  |
| 4 | 15-13 | `category` | enum | enum `turbulence_category` |  |
| 4 | 12-0 | `spare_4` | spare |  |  |
| 5 | 15-0 | `altitude` | uint | LSB 1 ft, range 0 to 65535 | Beam height at the bin: range x tan(lowest elevation) |

## Where every legacy message type went

All 89 `MESSAGE_SCHEMAS` types from `FMOFP/MIL_STD_1553B/message_schemas.py`.

| Legacy type | Now |
|---|---|
| `aewc_radarCommand` | RDR_AEWC_MODE_CMD |
| `aewc_radarData` | block RDR_AEWC_SECTOR |
| `aewc_radarModeChangeRequest` | RDR_AEWC_MODE_CMD |
| `aewc_radarModeChangeResponse` | RDR_AEWC_STATUS |
| `aewc_radarSectorScanRequest` | RDR_AEWC_MODE_CMD |
| `aewc_radarSectorScanResponse` | block RDR_AEWC_SECTOR |
| `aewc_radarStatusRequest` | RDR_AEWC_STATUS |
| `aewc_radarStatusResponse` | RDR_AEWC_STATUS |
| `bitCommand` | mode code 3 |
| `bitDataResponse` | deferred: v2: detailed BIT results as a block per terminal |
| `bitStatusRequest` | mode code 19 |
| `bitStatusResponse` | mode code 19 |
| `display_data` | retired: As displayDataUpdate |
| `display_mode_change` | DSP_PFD_MODE_CMD; DSP_MFD_MODE_CMD; DSP_EICAS_MODE_CMD; DSP_RADAR_MODE_CMD; DSP_TSD_MODE_CMD; DSP_SMS_MODE_CMD; DSP_HUD_MODE_CMD |
| `display_mode_request` | DSP_PFD_MODE_CMD; DSP_MFD_MODE_CMD; DSP_EICAS_MODE_CMD; DSP_RADAR_MODE_CMD; DSP_TSD_MODE_CMD; DSP_SMS_MODE_CMD; DSP_HUD_MODE_CMD |
| `display_mode_response` | DSP_PFD_STATUS; DSP_MFD_STATUS; DSP_EICAS_STATUS; DSP_RADAR_STATUS; DSP_TSD_STATUS; DSP_SMS_STATUS; DSP_HUD_STATUS |
| `DISPLAY_VIL_DATA` | block RDR_WX_PRODUCTS |
| `displayCommand` | DSP_PFD_MODE_CMD; DSP_MFD_MODE_CMD; DSP_EICAS_MODE_CMD; DSP_RADAR_MODE_CMD; DSP_TSD_MODE_CMD; DSP_SMS_MODE_CMD; DSP_HUD_MODE_CMD |
| `displayDataUpdate` | retired: Displays receive the producer's own messages through BC-scheduled RT-to-RT transfers |
| `displayStatusRequest` | DSP_PFD_STATUS; DSP_MFD_STATUS; DSP_EICAS_STATUS; DSP_RADAR_STATUS; DSP_TSD_STATUS; DSP_SMS_STATUS; DSP_HUD_STATUS |
| `displayStatusResponse` | DSP_PFD_STATUS; DSP_MFD_STATUS; DSP_EICAS_STATUS; DSP_RADAR_STATUS; DSP_TSD_STATUS; DSP_SMS_STATUS; DSP_HUD_STATUS |
| `flight_control_systemCommand` | FCS_MODE_CMD |
| `flight_control_systemControlInputRequest` | FCS_SURFACE_CMD |
| `flight_control_systemControlInputResponse` | FCS_SURFACE_STATUS |
| `flight_control_systemModeChangeRequest` | FCS_MODE_CMD |
| `flight_control_systemModeChangeResponse` | FCS_STATUS |
| `flight_control_systemOrientationDataRequest` | FCS_ATTITUDE |
| `flight_control_systemOrientationDataResponse` | FCS_ATTITUDE |
| `flight_control_systemStatusRequest` | FCS_STATUS |
| `flight_control_systemStatusResponse` | FCS_STATUS |
| `flightManagementSystemCommand` | FMS_MODE_CMD |
| `flightManagementSystemStatus` | FMS_STATUS |
| `fms_attitudeUpdateRequest` | FMS_ATTITUDE_CMD |
| `fms_attitudeUpdateResponse` | FMS_ATTITUDE_STATUS |
| `fms_maneuverRequest` | FMS_MANEUVER_CMD |
| `fms_maneuverResponse` | FMS_MANEUVER_STATUS |
| `fms_modeChangeRequest` | FMS_MODE_CMD |
| `fms_modeChangeResponse` | FMS_STATUS |
| `fms_navigationUpdateRequest` | FMS_NAV_UPDATE_CMD |
| `fms_navigationUpdateResponse` | FMS_NAV_UPDATE_STATUS |
| `fmsCommand` | FMS_MODE_CMD |
| `fmsDataRequest` | FMS_FLIGHT_DATA |
| `fmsDataResponse` | FMS_FLIGHT_DATA |
| `fmsStatusRequest` | FMS_STATUS |
| `fmsStatusResponse` | FMS_STATUS |
| `radarCommand` | RDR_WX_MODE_CMD; RDR_TFR_MODE_CMD; RDR_SAR_MODE_CMD; RDR_TGT_MODE_CMD; RDR_AEWC_MODE_CMD |
| `radarDataRequest` | block RDR_WX_PRODUCTS |
| `radarDataResponse` | block RDR_WX_PRODUCTS |
| `radarStatusRequest` | RDR_WX_STATUS; RDR_TFR_STATUS; RDR_SAR_STATUS; RDR_TGT_STATUS; RDR_AEWC_STATUS |
| `radarStatusResponse` | RDR_WX_STATUS; RDR_TFR_STATUS; RDR_SAR_STATUS; RDR_TGT_STATUS; RDR_AEWC_STATUS |
| `sar_radarCommand` | RDR_SAR_MODE_CMD |
| `sar_radarData` | block RDR_SAR_IMAGE |
| `sar_radarImageRequest` | RDR_SAR_MODE_CMD |
| `sar_radarImageResponse` | block RDR_SAR_IMAGE |
| `sar_radarModeChangeRequest` | RDR_SAR_MODE_CMD |
| `sar_radarModeChangeResponse` | RDR_SAR_STATUS |
| `sar_radarStatusRequest` | RDR_SAR_STATUS |
| `sar_radarStatusResponse` | RDR_SAR_STATUS |
| `systemCommand` | TERM_CMD |
| `systemDataRequest` | deferred: v2: each terminal's periodic data messages replace generic data requests |
| `systemDataResponse` | deferred: As systemDataRequest |
| `systemStatusRequest` | TERM_STATUS |
| `systemStatusResponse` | TERM_STATUS |
| `targeting_radarCommand` | RDR_TGT_MODE_CMD |
| `targeting_radarData` | block RDR_TGT_TRACKS |
| `targeting_radarModeChangeRequest` | RDR_TGT_MODE_CMD |
| `targeting_radarModeChangeResponse` | RDR_TGT_STATUS |
| `targeting_radarStatusRequest` | RDR_TGT_STATUS |
| `targeting_radarStatusResponse` | RDR_TGT_STATUS |
| `targeting_radarTrackRequest` | RDR_TGT_MODE_CMD |
| `targeting_radarTrackResponse` | block RDR_TGT_TRACKS |
| `tfr_radarCommand` | RDR_TFR_MODE_CMD |
| `tfr_radarData` | block RDR_TFR_ELEVATION |
| `tfr_radarElevationDataRequest` | RDR_TFR_MODE_CMD |
| `tfr_radarElevationDataResponse` | block RDR_TFR_ELEVATION |
| `tfr_radarModeChangeRequest` | RDR_TFR_MODE_CMD |
| `tfr_radarModeChangeResponse` | RDR_TFR_STATUS |
| `tfr_radarStatusRequest` | RDR_TFR_STATUS |
| `tfr_radarStatusResponse` | RDR_TFR_STATUS |
| `weather_radarCommand` | RDR_WX_MODE_CMD |
| `weather_radarData` | block RDR_WX_PRODUCTS |
| `weather_radarModeChangeRequest` | RDR_WX_MODE_CMD |
| `weather_radarModeChangeResponse` | RDR_WX_STATUS |
| `weather_radarPrecipitationRequest` | RDR_WX_MODE_CMD |
| `weather_radarPrecipitationResponse` | block RDR_WX_PRODUCTS |
| `weather_radarStatusRequest` | RDR_WX_STATUS |
| `weather_radarStatusResponse` | RDR_WX_STATUS |
| `weather_radarVILRequest` | RDR_WX_MODE_CMD |
| `weather_radarVILResponse` | block RDR_WX_PRODUCTS |

