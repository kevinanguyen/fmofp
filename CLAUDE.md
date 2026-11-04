# FMOFP Python-to-C++ port: the shared working file

**Read this whole file before doing anything else in this repository.** Two people take
turns on this fork, Kevin (the owner) and his partner, each driving Claude (Fable 5.1) in
their own sessions, on their own machines, at different times. This file is how one session
hands the work to the next. It is checked in on branch `port/cpp`, Claude Code loads it
automatically, and **the copy on disk is only as current as the last `git pull`**, so the
first thing every session does is pull (section 0). The last thing every session does is
update this file and push it with the work (section 0.4). A session that does not push is
invisible to the other person; a session that does not pull may redo or undo their work.

Section numbers are anchors both agents rely on: add to sections, do not renumber them.
Sections 7 and 8 are append-only.

Contents: 0 session protocol · 1 current state · 2 the units (progress meter, with the open
units' details) · 3 how a unit is ported · 4 rules · 5 machines, tools, commands · 6 gotchas
· 7 unit log · 8 decisions.

---

## 0. Session protocol (mandatory for both agents)

### 0.1 Start of every session, before reading code or answering anything about the project

1. `git status --short` must print nothing. If it does not, report it to your human and do
   not stash, reset or commit it blindly: it may be the other person's unfinished work on a
   shared machine, or your own from a session that died.
2. `git fetch origin && git fetch upstream`, then `git checkout port/cpp`, then
   `git pull --ff-only origin port/cpp`. If the pull is not a fast-forward, someone committed
   locally without pushing: stop and ask.
3. If the pull changed this file (`git diff --stat ORIG_HEAD HEAD -- CLAUDE.md` prints a
   line), **read this file again in full**: the copy Claude Code loaded at session start is
   stale.
4. Read section 1 (current state) and the last entries of section 7 (unit log). Look for any
   unit marked `IN PROGRESS` or any `wip/` branch named in section 1: that is the other
   person's open work.
5. `git log --oneline HEAD..upstream/main`: if it prints anything, Alee's upstream has
   moved; apply the merge rule (section 4.4) before starting a unit.
6. On a machine's first session: do the setup of section 5 and run `bash B20SS/verify.sh`
   until it is green before changing anything. Then say so in section 7.

### 0.2 Claim a unit before building it

Set the unit's status in the section 2 table to `IN PROGRESS (<name>, <date>)`, commit that
one-line change ("Claim unit N: <name>") and push. It costs a minute and keeps the two of you
from porting the same subsystem. If a unit is already claimed by the other person and the
claim is less than two days old, take another unit or ask your human. A claim older than that
with no `wip/` branch and no log entry is stale: say so in section 7 and take the unit.

### 0.3 While working

One unit at a time, done to the standard of section 3: code, tests, ICD, docs and this file
together. Do not start a second unit in the same commit. Record decisions as they are made
(section 8), not at the end.

### 0.4 End of every turn that changed anything (that is, after every prompt)

1. **Update this file**: section 1 (current state block), the unit's row in section 2, a
   section 7 entry (what was done, how, warnings and lessons, files, tests, numbers), new
   gotchas in section 6, any decision in section 8, and the details of an open unit if you
   learned something about it.
2. **Verify**: `bash B20SS/verify.sh` (section 5.5) must be green. For a change that touches
   only documents, the `icd` step is enough. Never push red to `port/cpp`.
3. **Commit and push**: `git add` the files you changed (never `git add -A` without reading
   `git status`), commit with a message in the style of section 4.6, then
   `git push origin port/cpp`. If the push is rejected, `git pull --rebase origin port/cpp`,
   re-run `verify.sh` if any code conflicted, push again. Never force-push `port/cpp`.
4. **Tell your human** in plain words: what was done, the commit id, the conversion
   percentage with the time left (section 4.7), what is next, and the one decision they must
   make, if any.

A turn that changed nothing in the repository (a question answered) needs no commit: say so.
A decision made in conversation **is** a change: write it in section 8 and push.

Unfinished work never stays on `port/cpp`. If a unit cannot be brought to green before the
session ends, commit it on a branch `wip/<unit>` (tests may be red there), push that branch,
and record in section 1 and section 7 what works, what does not, and the next step. Either
person may continue it in their next session; when it is green it is rebased onto
`port/cpp` (one commit) and the `wip/` branch deleted.

### 0.5 Never

- Never push to `upstream` (TubAleeLo/fmofp, Alee's repository). Keep its push URL disabled:
  `git remote set-url --push upstream DISABLED`.
- Never force-push or rewrite history on `port/cpp`.
- Never edit the frozen Python under `B20SS/FMOFP/` except by merging upstream (section
  4.4). The C++ is measured against that Python.
- Never commit build directories, logs, databases or anything under `B20SS/logs/`.
- Never put the other person's or your own secrets, tokens or passwords anywhere in the
  repository, including this file.
- Never delete or rewrite entries of sections 7 and 8; append corrections.

---

## 1. Current state (rewrite this block; keep it short and exact)

- **Branch** `port/cpp` on `origin` = github.com/kevinanguyen/fmofp. HEAD: see `git log -1`.
  Last milestone commit: `051e57d` (mission computer + comms, ICD v1.16; CI run
  41569356013 all ten jobs green). After it: `d82173f` merge of upstream/main `4c25f52`
  (six commits of 2026-11-02, mechanical; full suite green on the merged tree), then the
  commit that added this file and `B20SS/verify.sh`.
- **ICD** v1.16 (`B20SS/icd/fmofp_icd.xml`): 26 terminals, 102 message definitions
  (148 after per-terminal expansion), 35 message relays (15 broadcast), 20 block streams
  (10 layouts, 10 relayed copies), 9 mode codes, 89/89 legacy types mapped.
- **C++ partitions (processes)**: FCS (RT 5), nav (RT 7), FMS (RT 12), radar computer
  (RT 9: weather, terrain-following, targeting + track fusion, AEWC, synthetic aperture),
  engine management (RT 3), mission computer (RT 6), communications (RT 2); the bus
  controller `fmofp_bc`; tools `fmofp_nav_tool`, `fmofp_radar_tool`, `fmofp_engine_tool`,
  `fmofp_mission_tool` (stdin oracles for the Python tests).
- **Python test equipment on the bus**: the plant `py -m fmofp_plant` (RT 27: the aircraft's
  random-walk flight state, GPS pseudoranges, weather, terrain, two engines, air traffic, the
  ground scene, the RF environment); six display terminals `py -m fmofp_display
  radar|pfd|mfd|eicas|tsd|sms` (RT 15, 11, 13, 14, 16, 17), each an unchanged FMOFP display
  class fed from the bus through `fmofp_display/*_feed.py`; the one-command demo
  `py -m fmofp_demo`; the bus library `fmofp_bus` with generated `fmofp_bus/icd/`.
- **Load**: 14 terminals, 77 scheduled messages, worst minor frame 6908 us of the 10000 us
  budget. Forty seconds of headless demo: 2000 frames, 0 overruns, 14/14 terminals answering.
- **Tests**: Python 186 OK (`fmofp_bus/tests`, 35 s on Windows); C++ ctest 2/2
  (hand-written gtests + generated ICD round-trip tests), cppcheck clean; CI (`.github/
  workflows/ci.yml` job `cpp`, linux-gcc + linux-asan, and Alee's eight Python legs).
- **Meter**: 21 of 29.5 units = **71 %** (section 2). Pace measured over the log: about
  5 units per full working day, so roughly two more working days to 100 %; the intern-ticket
  authoring phase comes after and is not in the number.
- **Upstream**: merged through `4c25f52` (2026-11-02). Port baseline for the ported logic
  stays `0ab9236` (Alee's freeze, 2026-10-10); later upstream commits have been plumbing and
  lint only.
- **Open work / claims**: none. No `wip/` branches. Last session: Kevin's, 2026-11-04 (this
  file, the merge). Suggested next unit: 18 defensive (RT 22), then 15 sensor management,
  16 power, 17 avionics, 19 flight surfaces, 20 utilities, 28, 29 last, 30.
- **Known red in CI that is not ours**: Alee's `test_weather_radar_live` is flaky on the
  Windows and 3.12 Python legs; the two `cpp` jobs are the ones that judge the port.

---

## 2. The units (progress meter)

Percentage = done weight / 29.5. Weights are rough equal-effort units; change one only with
a reason written in section 8. Status values: `open`, `IN PROGRESS (<name>, <date>)`,
`DONE <date>` (with the section 7 entry), `STRUCK <date>` (with the decision in section 8).

| # | Unit | Weight | Status |
|---|---|---|---|
| 1 | Bus core: words, RemoteTerminal, bus controller, transport, OSAL | 2 | DONE 2026-10-10 |
| 2 | ICD + C++/Python generators + checks | 1 | DONE 2026-10-10 |
| 3 | Python bus library + plant framework + monitor | 1 | DONE 2026-10-10 |
| 4 | FCS partition (RT 5) | 1 | DONE 2026-10-10 |
| 5 | Nav partition (RT 7) | 1 | DONE 2026-10-11 |
| 6 | FMS partition (RT 12) | 1.5 | DONE 2026-10-11 |
| 7 | Weather radar (RT 9) | 2 | DONE 2026-10-11 |
| 8 | Terrain-following radar | 1 | DONE 2026-10-25 |
| 9 | Engine management (RT 3) | 1 | DONE 2026-10-25 |
| 10 | Targeting radar + track fusion | 1.5 | DONE 2026-11-01 |
| 11 | AEWC radar | 1 | DONE 2026-11-01 |
| 12 | SAR radar | 1 | DONE 2026-11-01 |
| 13 | Mission computer (RT 6) | 1 | DONE 2026-11-02 |
| 14 | Communications (RT 2) | 0.5 | DONE 2026-11-02 |
| 15 | Sensor management (RT 10) | 0.75 | open |
| 16 | Power management (RT 8) | 0.75 | open |
| 17 | Avionics: LRU health + BIT + fitness + SW configuration (RT 1) | 1 | open |
| 18 | Defensive systems (RT 22) | 0.5 | open |
| 19 | Flight surfaces (RT 21) | 0.5 | open |
| 20 | Small utilities: ECS (RT 4), hydraulics (RT 19), airframe (RT 20), flight data monitoring (RT 23) | 1 | open |
| 21 | Radar display + PFD on the bus | 1.5 | DONE 2026-10-25 |
| 22 | MFD on the bus | 1 | DONE 2026-10-25 |
| 23 | Broadcast relays | 0.5 | DONE 2026-10-25 |
| 24 | EICAS on the bus | 0.5 | DONE 2026-11-01 |
| 25 | TSD on the bus | 0.5 | DONE 2026-11-01 |
| 26 | SMS on the bus | 0.5 | DONE 2026-11-02 |
| 27 | HUD on the bus | 0.5 | STRUCK 2026-11-02 (the Python HUD is an empty placeholder; removed from the total) |
| 28 | Scenario engine + debug CLI on the bus | 1.5 | open |
| 29 | Retire the old Python messaging stack and the radar bridge | 1 | open |
| 30 | Hardening: release build, timing checks, coverage targets, MISRA-subset lint, intern docs | 1.5 | open |

**Done: 21 / 29.5 = 71 %.** Open: 8.5 units.

The terminal addresses are fixed in the ICD's terminal table (`fmofp_icd.xml`, `<terminals>`):
RT 1 avionics, 2 comms, 3 engine, 4 ECS, 5 FCS, 6 mission, 7 nav, 8 power, 9 radar, 10 sensor
management, 11 PFD, 12 FMS, 13 MFD, 14 EICAS, 15 radar display, 16 TSD, 17 SMS, 18 HUD
(unused), 19 hydraulics, 20 airframe, 21 flight surfaces, 22 defensive, 23 flight data
monitoring, 27 plant, 31 broadcast. Free: 24-26, 28-30.

### 2.15 Sensor management (RT 10), 0.75

- **Python to read in full**: `B20SS/FMOFP/Systems/sensorManagement/sensorService.py`
  (238 lines; `SensorService` polls at 5 Hz: the statuses of the active sensors (radar,
  lidar) and passive sensors (infrared, ESM) which `activate_all()` turns on at start, the
  fused track count from `RadarDataFusion`, `SensorDataFusion.update()` and its contacts;
  health NOMINAL when all active and all passive sensors are active, DEGRADED when one group
  is, FAULT otherwise), `sensorDataFusion.py` (349: `SensorContact`; `_ingest_active` /
  `_ingest_passive` read the managers' `scan()`/`detect()`; `_merge_or_add` merges by source
  and bearing delta; `_expire_stale` by wall clock; `get_contacts`, `get_summary`),
  `activeSensors/activeSensors.py` (150) and `passiveSensors/passiveSensors.py` (137): the
  detections are random numbers (`random.uniform` bearings and frequencies). `smConfig.xml`
  is a one-line placeholder.
- **Consumers today**: the EICAS poll reads `get_sensor_service().get_data()` and uses only
  `health` and `fused_track_count` (`eicas.py` around line 359). Nothing draws the sensor
  contacts. `fmofp_display/systems_feed.py` `sensor_data()` returns `{}` for now (the EICAS
  shows UNKNOWN).
- **Shape of the port**: the random detections are the world, so they move to the plant
  (`fmofp_plant/`: a detection generator in the Python's draw order, published as a plant
  message or block; the defensive unit wants the same ESM detections, so publish them once
  and relay to both RT 10 and RT 22, or broadcast if both exist: two listeners qualify). The
  service's rules (sensor statuses, the health rule, the fusion's merge/expiry, the counts)
  become the RT 10 partition. Messages: `SNS_STATUS` (health, active/passive bits, contact
  counts; 1 or 5 Hz), `SNS_CMD` (activate/deactivate a sensor), a contacts block only if a
  display will draw it (today none would). The EICAS is the only consumer of the status, so
  it comes by relay into an EICAS receive SA (section 4.2 rule). For the fused track count,
  prefer letting the EICAS count the fused-tracks block it already receives rather than
  making RT 10 a listener of it; record the choice in the ICD README section 2.
- **Tests**: a `fmofp_sensor_tool` oracle against the frozen `SensorDataFusion` with its
  `time` patched (as `test_targeting_oracle.py` and `test_mission_oracle.py` do), an e2e
  run with the plant, the EICAS feed and render tests (the health row), the demo dashboard.
- **Traps**: the `handle_message` XML paths in both managers are dead; the fusion's expiry
  and ids use wall clock and a counter (frames and a counter in C++); keep the Python's
  draw order for the detections so the oracle can follow.

### 2.16 Power management (RT 8), 0.75

- **Python to read in full**: `Systems/powerManagement/elec/powerManagementSystem.py`
  (319; 1 Hz loop: `adjust_power_parameters(dt_hours = 1/3600)` draws the generator output
  200-250 kW and the consumption 180-220 kW at random, the balance charges or discharges the
  main battery (amps = |balance| x 1000 / nominal volts), charges the APU battery at a
  quarter, the emergency battery sustains the critical bus, the cooling gets a heat load of
  6 x consumption, HVAC updates; `monitor()` snapshots the three battery charges, generator
  output, consumption, balance, bus voltages drawn 110-120 V and 25-28 V, frequency 398-402,
  power factor 0.95-1, coolant and cabin temperatures; `set_generator_output`;
  `health_report`), the batteries `batts/mainBatt.py` (109), `emergencyBatt.py` (65),
  `missionEquipmentBatt.py` (68), thermal `thermalManagement/coolingSystems/cooling.py` (71),
  `HVAC.py` (70), `thermalDissipation/activeDissipation.py` (86). `epConfig.xml` and
  `pmConfig.xml` are placeholders.
- **Consumers today**: the EICAS electrical rows, which read UNKNOWN until this terminal
  exists; `EicasWindow.tick()` hands readings to `eicas.set_system_readings(...)` (that is
  how the engine readings reached the panel in unit 24). No other display reads power.
- **Shape of the port**: the random draws (generator output, consumption, voltages,
  frequency, power factor) are the world: plant message `PLANT_ELECTRICAL` at 1 Hz in the
  Python's draw order. The bookkeeping (battery charge and discharge per step, the APU and
  emergency batteries, the thermal loads) is the RT 8 partition; `PWR_STATUS` carries what
  `monitor()` snapshots; `PWR_CMD` for `set_generator_output` and any load shedding. One
  consumer (EICAS): relay.
- **Tests**: oracle = the frozen battery and cooling classes stepped with the same inputs
  (they are deterministic once the random draws are inputs); feed and render tests for the
  EICAS electrical rows.
- **Traps**: `monitor()` re-draws the voltages on every call, so they are not state;
  `receive_message()` and `_process_received_message()` are dead; the 1 Hz loop's dt is a
  constant 1/3600 h, not the measured time.

### 2.17 Avionics: LRU health, built-in test, FM fitness, software configuration (RT 1), 1

- **Python to read in full**: `Systems/avionics/hardwareHealth/LRUstatus.py` (449;
  `LRUStatusMonitor` at 1 Hz: `_build_catalogue()` maps each LRU to a live subsystem's getter
  (`_acc(import_path, func)`), `_derive_health(status, lru)` turns a status dict into a
  `HealthState`, forced faults for the scenario engine (`force_fault`, `clear_forced_fault`,
  `get_forced_faults`), `get_faults`, `overall_health`, `update_bit_result`),
  `builtInTesting.py` (297; `PowerOnSelfTest`, `PeriodicBIT` whose intervals are minutes
  from the config, `InitiatedBIT`; `_probe_system` pokes a live subsystem),
  `Systems/builtInTestSystems/bitControl.py` (153; `BuiltInTestController` parses
  `bitsConfig.xml` (72 lines: self tests, periodic tests with intervals, interface tests) and
  runs them), `Systems/fmFitness/fmStatus.py` (155; `FlightManagementFitness` at 1 Hz:
  `check_thresholds(component, metrics)` from `fmfConfig.xml` (33 lines),
  `handle_redundancy`), `Systems/configurationManagement/swConfigure.py` (121;
  `SoftwareConfigManager` reads `swcmConfig.xml` (52 lines: components, versions, data
  loads); `run()` applies one hard-coded data load "20191015"; it has no thread).
- **Consumers today**: the EICAS lists `BuiltInTestController().self_tests` as "id: PASS"
  strings (`eicas.py` around 343; `SystemsFeed.bit.self_tests = []` for now); the scenario
  engine's failure injection calls `LRUStatusMonitor.force_fault` (unit 28 will send it as a
  command). The scenario engine's forced fault is the only path that makes an LRU unhealthy.
- **Shape of the port**: on the bus the LRU monitor cannot call other subsystems' getters.
  Its inputs are what the bus controller already knows: every terminal's TERM_STATUS reply
  (state, BIT word) and its poll statistics. The clean design is a bus-health message from
  the bus controller (per RT: state, BIT word, answered/no-response counts) that RT 1
  receives at 1 Hz; RT 1 derives the LRU table from it, applies forced faults, runs the BIT
  schedule (power-on, periodic, initiated), the fitness thresholds and the software
  configuration table, and publishes `AVN_STATUS` (overall health, fault count, BIT
  summary) and an `AVN_LRU` block (per LRU: health, last BIT result, forced) for the EICAS,
  with `AVN_CMD` (initiated BIT; force/clear a fault; apply a data load). This touches the
  bus controller (`B20SS/ofp/bc/`): keep that change small, with its own test in
  `bus_controller_test.cpp`.
- **Tests**: oracle = `_derive_health`, `check_thresholds` and `PeriodicBIT.run_due` with a
  patched clock; the EICAS BIT list in the feed and render tests; a scripted forced fault
  end to end.
- **Traps**: `_probe_system` and `_build_catalogue` import every live subsystem (do not port
  the probe, port the catalogue and the rules; construct the monitor in the oracle without
  starting it); periodic intervals are 90 minutes (nothing happens in a test without a clock
  you control); `swConfigure.run()` is a one-shot hard-coded load; `avConfig.xml` is a
  placeholder.

### 2.18 Defensive systems (RT 22), 0.5

- **Python to read in full**: `Systems/defensiveSys/defensiveService.py` (362; a 2 Hz
  loop; `THREAT_DB` and `_classify_freq` at the top; `_update_rwr`: one contact per fusion
  threat track (`FUS-<id>`: bearing and range from the track, band/type/priority from
  `THREAT_DB` by the track's classification, hostile from the track) and one per ESM signal
  from its own `PassiveSensorManager` (`ESM-<band>-<bearing>`: a random range 5-60 nm, type
  UNK, priority 3, hostile); contacts older than `TRACK_TTL` 10 s are dropped;
  `_auto_dispense`: a priority-1 hostile contact releases `SALVO_SIZE` 2 flares with a
  `COOLDOWN_S` 3 s; manual `dispense_chaff` (`CHAFF_MAX` 60), `dispense_flares`
  (`FLARE_MAX` 30, the same cooldown), `set_ecm_mode` STANDBY/ACTIVE/PASSIVE (jamming active
  only in ACTIVE); `get_rwr_contacts()` sorted by priority; `get_data()`: chaff, flares,
  ecm_mode, jamming_active, rwr_contacts, threat_count). `dsConfig.xml` is three lines.
- **Consumers today**: the TSD draws `get_rwr_contacts()` on its threat ring using
  `bearing_deg` and `band` (`tsd.py` around 554); the EICAS reads `get_data()` for its
  countermeasures cells (read `eicas.py` for the keys it draws). The SMS's ECM cell comes
  from the FMS tactical status, not from here (kept that way in unit 26).
  `SystemsFeed.defensive_data()` returns `{}` and `get_rwr_contacts()` `[]` for now.
- **Shape of the port**: RT 22 is a listener of the fused-tracks broadcast
  (`BCAST_RDR_FUSED_TRACKS`, RT 31 SA 22-23, as the TSD and EICAS are; the records carry
  the class, identity, threat and position the Python read from the track objects); the ESM
  signals come from the plant (shared with unit 15 if it exists by then, else a small
  `PLANT_ESM` message now, in the Python's draw order). Outputs: `DEF_STATUS` at 2 Hz
  (chaff, flares, ECM mode, jamming, contact count, a dispense counter) and the RWR contacts
  (a fixed message of up to eight contacts x 3 words, or a block), both with two listeners
  (TSD, EICAS) so broadcast on free SAs; `DEF_CMD` (chaff n, flares n, ECM mode) for the
  scenario engine and the CLI. Cooldowns and TTLs in frames.
- **Tests**: oracle = the frozen `DefensiveService` fed a fake fusion object and a fake ESM,
  with `time` patched; e2e with the plant's traffic making hostile fused tracks (the targeting
  e2e script shows how); TSD and EICAS feed/render tests; the demo dashboard (a `DEF` line)
  and an event (a flare salvo when the SAM appears).
- **Traps**: the ESM branch draws `random` inside the service (the randomness must come from
  the plant); the `THREAT_DB` keys must match the fused track class names exactly;
  `_log_event` writes to SQLite (drop); the auto-dispense reads wall clock.

### 2.19 Flight surfaces (RT 21), 0.5

- **Python to read in full**: `Systems/flightSurfaces/flightSurfaceService.py` (292; a
  50 Hz loop; `Surface.step(dt)` moves each surface toward its command at a rate limit;
  `_sync_from_fcc()` takes the commanded positions from the flight control computer (on the
  bus that is `FCS_SURFACE_STATUS`); `set_flaps(detent)`, `set_speedbrake(pct)`,
  `set_rudder(deg)`, `set_trim(axis, deg)`; `_persist` to SQLite; `get_data()` the positions;
  `health_report`). `fsConfig.xml` is three lines.
- **Consumers today**: none draws it. The natural consumer is the FCS itself: today
  `FCS_SURFACE_STATUS` reports commands, never measured positions.
- **Shape of the port**: RT 21 receives `FCS_SURFACE_STATUS` by relay, moves the surfaces at
  the Python's rates (50 Hz = every minor frame), publishes `FSF_POSITIONS` (measured
  positions with SSMs) relayed to the FCS, which reports them beside its commands (one new
  field group, or a second message; keep `FCS_SURFACE_STATUS` compatible), and takes
  `FSF_CMD` for flaps, speedbrake, rudder and trim. Smallest unit after the HUD: keep it so.
- **Tests**: oracle = `Surface.step`; a C++ test for the rate limit; e2e: command a surface
  through `FCS_SURFACE_CMD` and watch the position arrive over a few frames.

### 2.20 Small utilities, 1 (ECS RT 4, hydraulics RT 19, airframe RT 20, flight data monitoring RT 23)

Four thin terminals, one unit. Each is a small class with `init/step/status`, a status
message, and a plant source wherever the Python invented its readings. The EICAS gets its
remaining UNKNOWN rows from them (hydraulics C, ECS; electrical is unit 16) through
`set_system_readings` in `EicasWindow.tick()`.

- **ECS** `Systems/enviornmentalControlSystem/ecsControl.py` (199; 2 Hz; readings are
  hard-coded unless `set_sensor_readings` is called; `adjust_temperature/pressure/
  air_quality` nudge toward the targets of `ecsConfig.xml` (27 lines); `climate/
  climateControl.py` 41 and `oxygenGenerationsys/oxygenControl.py` 42 are stubs). Cabin
  environment to the plant (`PLANT_CABIN`), the regulation to RT 4, `ECS_STATUS` to the EICAS.
- **Hydraulics** `Systems/hydraulics/hydrControl.py` (187; 1 Hz; `monitor_pressure` over
  systems A/B/C against `hydrConfig.xml` (35 lines: nominal and minimum pressures),
  `activate_backup`, `health_report`). Pressures to the plant (`PLANT_HYDRAULICS`), the
  low-pressure and backup logic to RT 19, `HYD_STATUS` to the EICAS (whose A/B rows come
  from a g-load model today, C is unknown: the terminal's values replace the model).
- **Airframe** `Systems/airframeSystemManagement/airframeControl.py` (176; 1 Hz;
  `monitor_airframe` compares sensor readings with `asmConfig.xml` (38 lines) limits;
  `control_landing_gear(command)`, which the Python calls with "deploy" on every tick: a
  known defect; the port follows the FMS gear command in `FMS_CONTROL_STATUS` instead and
  says so in the ICD README section 2). Loads and temperatures to the plant, the limit
  checks and the gear state to RT 20, `AFR_STATUS`.
- **Flight data monitoring** `Systems/flightDataMonitoring/fdmControl.py` (148; a 2 Hz
  `_record_loop`: `collect_data()` snapshots other subsystems into the recorder,
  `eject_storage(slot)`; `fdmsConfig.xml` 53 lines lists the recorded parameters). On the bus
  RT 23 listens to the broadcasts it records (FMS flight data, engine status, GCAS alerts)
  into a fixed ring buffer and publishes `FDM_STATUS` (record count, slot state); a dump
  through the monitor or a command for the CLI (unit 28). No display reads it.

### 2.28 Scenario engine + debug CLI on the bus, 1.5

- **Python to read in full**: `Interfaces/scenarios/scenarioEngine.py` (395; loads
  `trainingScenario.xml` (126) or `failureScenario.xml` (106), sorts events by `time_s`, a
  daemon thread dispatches them; injectors `_inject_radar_contact` (writes a radar's
  `current_targets`), `_inject_failure` (an LRU forced fault), `_inject_weather` (a cell into
  the weather state manager), `_inject_threat` (RWR), `_inject_waypoint`, `_inject_phase`,
  `message` (log only)) and `Utils/debug/userCLI.py` (1031; a click REPL on four threads:
  `status`, `send`, `msg`, `test`, `test_1553b`, `scenario`, `help`).
- **Shape of the port**: both stay Python and become bus clients, like the plant and the
  displays. The scenario engine reads the same XML and sends bus commands at the right frame:
  waypoints and phases as `MSN_CMD`, radar contacts, weather cells and threats as
  injections into the plant (new plant *receive* messages, `PLANT_INJECT_*`, since the world
  lives there), failures as the avionics command of unit 17. The CLI becomes a REPL over the
  monitor port (status = decode the latest TERM_STATUS and status messages) and a command
  path. Both need one new thing in the bus controller: a command socket through which a
  Python client queues an aperiodic transaction (`fmofp_bc --script FILE` already queues
  commands by frame; generalise it to a port and add a `fmofp_bus` client). Design it first,
  write it in the ICD README section 7, then build. Both scenario files must run end to end
  over the bus in a test. The demo's scripted events (`fmofp_demo/demo.py`) are the model for
  the engine's output.

### 2.29 Retire the old Python messaging stack and the radar bridge, 1

Delete what the bus replaced: `B20SS/FMOFP/local_messaging/` (about 38,700 lines),
the Python bus `B20SS/FMOFP/MIL_STD_1553B/` (19,300), `radar_to_display_bridge.py` (695),
the display-side mirror `Interfaces/userInterface/messaging/`, the per-subsystem message
handlers and messengers, their tests under `FMOFP/Tests/`, and the wiring in
`core/system_manager.py` / `Main.py` that starts the Python subsystems now running in C++.
Decide with the humans what `py FMOFP/Main.py` becomes (the demo launcher already is the
aircraft). Both CI files change (Alee's Python jobs test the deleted stack). **Do this unit
last among the port units**: Alee's upstream still edits these files every week, so after
this unit upstream merges will conflict and the merge rule (section 4.4) stops the pulling;
tell both humans that the upstream line is cut from then on.

### 2.30 Hardening, 1.5

A release preset (every preset builds Debug `-O0` today, demo binaries included); a timing
test that asserts each partition's step time and the schedule's worst frame against the
budget; gcovr thresholds per partition in CI; a MISRA-subset lint (cppcheck's misra addon,
or clang-tidy on the Linux job); the FMOFP loggers at WARNING in the display processes (the
MFD log grows 2 MB per three minutes); the intern documents: how to build, run and debug one
partition under gdb, the architecture page, the ticket format. Cheap backlog items from the
ICD README section 12 can ride along (a plant whose airspeed answers the thrust, engine
spool-up).

---

## 3. How a unit is ported (the pattern every done unit followed)

1. **Scope the Python in full.** Read the module(s), the config XML beside them, and every
   consumer (grep the displays and the other Systems for the getter). Write down, before
   designing: which parts are *world* (random walks, invented readings, generated detections,
   wall-clock sines) and which are *flight logic* (rules, thresholds, state machines, filters,
   bookkeeping); every quirk and defect you find; what each display actually reads (keys and
   units, not what the docstring says).
2. **World goes to the plant, logic goes to C++.** The plant (`B20SS/fmofp_plant/`) owns
   everything the real aircraft would measure; the partition owns everything the real
   computer would decide. The ported logic is translated step for step from the Python,
   including its quirks, unless the quirk makes the function unusable (then depart, and
   record the departure in `B20SS/icd/README.md` section 2 with the evidence). Improvements
   go to the README's section 12 backlog, not into the port.
3. **Design the bus interface in the ICD first** (`B20SS/icd/fmofp_icd.xml`): the
   terminal's status message and mode command (SA 10 by convention), its inputs (relays of
   plant or other terminals' messages, or broadcast listeners), its commands (aperiodic, with
   `cmd_seq`), blocks for anything over 32 words (`<block>` with a `<header>` and
   `<section>`s using `id=`). Apply the rules of section 4.2. Run
   `py icd/tools/icd_tool.py check`, bump the minor version, regenerate (`gen-py`,
   `render`), update the README: a section 1 paragraph for the version, section 2 rows for
   every departure from the Python, the "Added in vX" table, section 12, the revision row.
4. **Plant side** if any world moved: `fmofp_plant/<thing>.py` with its own seeded
   generator (`seed ^ a constant`, so earlier seeded flights do not change), `plant.py`
   publishing it at its rate and reading any command the partition sends back, CLI flags,
   `fmofp_bus/tests/test_plant_<thing>.py`.
5. **C++ partition**: `B20SS/ofp/partitions/<name>/{include/fmofp/<name>/<name>.hpp,
   src/<name>.cpp, src/main.cpp, tools/<name>_tool.cpp}` modelled on `partitions/comms`
   (the smallest) or `partitions/mission` (with a block and a tool); `partitions/
   CMakeLists.txt`; `tests/<name>_test.cpp` and `tests/CMakeLists.txt`; the counts in
   `tests/icd_messages_test.cpp` (`kIcdMinor`, messages, relays). Rules: `B20SS/ofp/
   CLAUDE.md`. The stdin tool exists so the Python oracle test can drive the C++ logic with
   the frozen Python's own inputs and compare outputs.
6. **Python side**: `fmofp_display/systems_feed.py` (stand-ins returning the dictionaries
   the display reads; omit a field until it is valid rather than inventing one),
   `fmofp_display/app.py` (which terminal receives which messages and blocks, with
   `accept_broadcast` for RT 31), `fmofp_demo/demo.py` (`PARTITIONS`, `NAMES`, a dashboard
   line, an event in time order with `cmd_seq` 201 or above, the banner), tests:
   `test_messages.py` counts and listeners, `test_display_feeds.py`, `test_display_render.py`
   (offscreen Qt), `test_display_e2e.py` (the whole aircraft, every terminal),
   `test_demo.py`, `test_<name>_oracle.py`, `test_<name>_e2e.py` (its own `PORT_BASE`, at
   most 65000, distinct from every other test's), the env var in `fmofp_bus/tests/
   _support.py` and in **both** CI files (`.github/workflows/ci.yml`, `bitbucket-pipelines.yml`),
   and `B20SS/verify.sh` `export_binaries` if a new binary or tool exists.
7. **Docs**: `B20SS/ofp/README.md` (the partition list, the command lines, the tests
   paragraph), the ICD README, this file (sections 1, 2, 7, 6, 8).
8. **Verify**: `bash B20SS/verify.sh` green; a headless demo run
   (`py -m fmofp_demo --headless --seconds 30`, from `B20SS/`) with 0 overruns; the load line
   under budget with margin.
9. **Commit, push, record** (section 0.4). One commit per unit, in the style of the
   existing ones (`git log` on `port/cpp` shows twenty).

---

## 4. Rules

### 4.1 Architecture

- The bus is the only link between programs: 16-bit words on a simulated MIL-STD-1553B over
  UDP on 127.0.0.1 (`port_base` + RT for terminals, +32 the controller, +33 the monitor).
  No shared memory, files or sockets of any other kind between partitions.
- One process per subsystem (a *partition*), one per display, the bus controller its own
  process and the master clock: a 20 ms minor frame, 50 per major frame; terminals step on
  the controller's synchronise mode code (MC17), never on their own clocks.
- The displays stay Python (PyQt6, unchanged FMOFP classes, fed from the bus). The scenario
  engine, the CLI, the plant and the tests stay Python. Everything that decides something
  about the aircraft is C++17 under `B20SS/ofp/`.
- The Python under `B20SS/FMOFP/` is frozen at Alee's `0ab9236` for the logic being ported;
  the C++ is measured against it by oracle tests that import the frozen modules. Upstream
  merges (section 4.4) are allowed only when they do not change that logic.
- The ICD is ours to evolve (Kevin's decision, 2026-10-10: no approvals through Alee).
  Decide, implement, record in the ICD README section 2, tell the humans.

### 4.2 Bus and ICD rules (the tool enforces most; know them anyway)

- Rates: 50, 25, 10, 5, 1 Hz or aperiodic. The controller refuses a schedule whose worst
  minor frame exceeds 10000 us; the load model charges a relay as one RT-to-RT transfer per
  consumer. Check the load line (`verify.sh load`) after every ICD change.
- **One consumer, its own relay** (v1.15): a stream with exactly one listener is relayed
  into that listener's own receive subaddress; RT 31 (broadcast) only for two or more
  listeners. A listener's own receive subaddresses must be broadcast numbers it never
  listens to (one receive buffer per SA); `check_broadcasts` refuses the collision. Free
  broadcast SAs after v1.16: 12 (not for the MFD), 16, 17, 20, 21, 30.
- Terminals store a broadcast only on subaddresses they `accept_broadcast`; everything else
  on RT 31 is ignored and counted.
- Blocks: 31 payload words per 32-word segment, descriptor + segments, CRC-16/CCITT-FALSE,
  version 1-255 (never 0). A block of n segments polled at rate r must be republished no more
  often than every 2n frames at 25 Hz and 5n frames at 10 Hz (the collector needs every
  segment once per version). `bus::BlockPublisher` must be `init(storage, capacity)`-ed
  before its first `publish`, or the descriptor reads zeros.
- Validity: every measured value carries a 2-bit SSM (0 = FAILURE_WARNING is the fail-safe
  zero; NO_COMPUTED_DATA for unknown or stale); staleness is counted in frames by the
  receiver. Commands carry an 8-bit `cmd_seq`; 0 means "no command"; the terminal reports
  the last accepted sequence and a rejected bit.
- Enums must define value 0 and fit in 8 bits; scaled fields give `lsb` as a fraction and a
  range; a plain unsigned field may not carry min/max without an lsb; block sections use
  `id=`. Message, relay and block counts are pinned in `icd_messages_test.cpp` and
  `test_messages.py`: update both with the ICD.
- Subaddress conventions: 29 TERM_STATUS/TERM_CMD on every terminal, 31 mode codes, 10 the
  terminal's status + mode command, radars on 1-5, displays' own status at 11-17; otherwise
  the next free number in the terminal's table.

### 4.3 C++ rules

`B20SS/ofp/CLAUDE.md` is binding (no heap after init, no exceptions, `[[nodiscard]]
Status`, `-Werror` with conversion warnings, cppcheck clean, tests for every module). Lessons
cppcheck taught: never name a local or parameter after a member accessor (`config`, `stats`,
`worst`, `mode`, `track`, `count`); prefer `std::find_if` / `count_if` / `fill` to raw search
loops; pointers to data you do not modify are `const`; a decoded record with no enum field
needs `(void)` on its decode status; `*_test.cpp` is suppressed (cppcheck cannot parse
`TEST(`). Shifts of promoted narrow integers trip `-Wsign-conversion` only under UBSan:
`static_cast<unsigned>` first. Large objects (the 4 MB radar volume) are `static` in
`main`, never stack temporaries. `layout_of` messages are composition (a `payload`
member), not inheritance.

### 4.4 Upstream (Alee's TubAleeLo/fmofp): merge when clean, otherwise report

Every session: `git fetch upstream`; `git log --oneline HEAD..upstream/main`. If there is
anything: `git diff --name-only $(git merge-base HEAD upstream/main) upstream/main`, and for
every file in the **mirrored set** (ICD sources: `rtAddressConfig.xml`, `message_schemas.py`,
`metadata_codec.py`, `radar_enums.py`, `messageRateConfig.xml`, `command_registry.xml`;
the modules the C++ was translated from; the modules the oracle tests import, listed by
`grep -h "from FMOFP" B20SS/fmofp_bus/tests/*.py B20SS/fmofp_display/*.py`; the display
classes the display processes embed) read the diff with `git diff -w`. Then
`git merge-tree --write-tree HEAD upstream/main` for conflicts. If conflict-free and the
mirrored files changed only in plumbing (imports, logging, comments, formatting): `git
merge --no-commit --no-ff upstream/main`, `bash B20SS/verify.sh icd py` (the oracle tests
arbitrate; run `cpp` too if anything under `ofp/` or `icd/` moved), commit the merge with a
message naming the upstream commit and what was checked, push. Otherwise `git merge
--abort`, and tell your human which files interfere; the baseline then stays where it is.
Never push to upstream.

### 4.5 The Python's quirks

Keep them as written and document them; they are the intern tickets of the future. Every
departure is a row in the ICD README section 2 with the reason and the evidence (an oracle
test that proves the Python's behaviour, where possible). Examples already recorded: the
storm-cell tracker that deletes every cell it creates, the SAR change detector that masks
its own detections, the AEWC radar equation mixing dBW and dBm, the mission route's ETEs
always at 450 kt, the SMS station table painted with the wrong brush.

### 4.6 Commits and CI

- One unit per commit, after `verify.sh` is green. Message: a one-line title naming the
  unit and the ICD version, then paragraphs saying what moved where, what the port settles
  about the Python, what the displays and the demo gained, and the tests, with numbers. End
  with the attribution line the harness gives you (`Co-Authored-By: Claude Fable 5.1
  <noreply@anthropic.com>` today).
- Both CI files are edited together (`.github/workflows/ci.yml` and
  `bitbucket-pipelines.yml`): the env vars for new binaries, the apt packages.
- After a push, note the CI run (`gh run list --repo kevinanguyen/fmofp --branch port/cpp
  --limit 3`; the `gh` binary on Windows is at `C:\Program Files\GitHub CLI\gh.exe` and
  needs `--repo`). Record the result in section 7 when it lands (next session is fine).
- Warnings like `LF will be replaced by CRLF` on commit are harmless (`-c
  core.safecrlf=false` silences them).

### 4.7 Reporting to the humans

Plain words, short, no jargon without a gloss (Kevin asked what a "port" and an "ICD" are:
explain such words the first time). Lead with what was done and whether it is verified;
give the commit id; **always give the conversion percentage from section 2 and the
estimated time to 100 % with the pace it rests on**; end with the one decision the human
has to make, if any. Report failures as failures with the output; never say "done" for a
step that was skipped.

### 4.8 Tooling rules for Claude on Windows (learned the hard way)

- Python is `py` (the launcher), never `python` (the Store shim). Install nothing globally
  without saying so.
- The C++ toolchain is MSYS2 UCRT64 at `C:\msys64`; it is not on the Windows PATH on
  Kevin's machine. `verify.sh` handles it; for anything else run
  `MSYSTEM=UCRT64 /c/msys64/usr/bin/bash.exe -lc 'cd <dir> && <command>'` or put
  `C:\msys64\ucrt64\bin` first on PATH (the test executables need its DLLs).
- **Never put a script in a Bash command body**: no heredocs (`<<'EOF'`), no inline Python
  with quotes, backslashes or escape sequences, no commit message inline. Write the file
  with the Write tool, then run `py file.py`, `bash file.sh` or `git commit -F file`. It has
  failed six times (length limits, apostrophes, doubled backslashes collapsed, escape
  sequences turned into newlines, variables lost in a backgrounded `&&` chain).
- PowerShell 5.1 mangles embedded quotes in native arguments: use the Bash tool for git,
  gh and the build.
- Python subprocesses want native paths in env vars on Windows (`K:/...`), not `/k/...`
  (`cygpath -m`). `taskkill /PID` gets path-converted under Git Bash (use PowerShell
  `Stop-Process -Id`); `wmic` is gone (`Get-CimInstance Win32_Process`).
- Scratch files, probes and patch scripts go in the session scratchpad, never in the
  repository.

---

## 5. Machines, tools, commands

### 5.1 Clone and remotes (once per machine)

```bash
git clone https://github.com/kevinanguyen/fmofp.git
cd fmofp
git checkout port/cpp
git remote add upstream https://github.com/TubAleeLo/fmofp.git
git remote set-url --push upstream DISABLED
git config user.name "<your name>" && git config user.email "<your email>"
```

The partner needs write access to `kevinanguyen/fmofp` (Kevin: repository Settings,
Collaborators). Until then the partner works on a fork of the fork and opens pull requests
into `port/cpp`, and the protocol's "push" means "push and open the PR".

### 5.2 Windows toolchain

Python 3.10 or newer from python.org with the `py` launcher, then
`py -m pip install -r B20SS/requirements.txt` (PyQt6 6.8.1, numpy, scipy, qasync, click).
MSYS2 (`winget install MSYS2.MSYS2`), then in the **UCRT64** shell (`C:\msys64\ucrt64.exe`):

```bash
pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-gtest mingw-w64-ucrt-x86_64-cppcheck mingw-w64-ucrt-x86_64-gcovr
```

Versions that are known to work: g++ 16.2, cmake 4.4, ninja 1.13, gtest 1.18, cppcheck 2.21,
gcovr 8.6. No sanitizers on MinGW: CI's `linux-asan` job covers that. Optional: GitHub CLI
(`winget install GitHub.cli`, then `gh auth login`).

### 5.3 Linux (what CI installs)

```bash
sudo apt-get install -y g++ cmake ninja-build libgtest-dev cppcheck gcovr python3 python3-numpy python3-scipy python3-pyqt6 libgl1 libegl1 libxkbcommon0 libfontconfig1 libdbus-1-3 libglib2.0-0
```

`verify.sh` picks `linux-gcc`; `PRESET=linux-asan bash B20SS/verify.sh cpp` runs the
sanitizer build. The Qt tests render offscreen (`QT_QPA_PLATFORM=offscreen`, set by the
script).

### 5.4 Build, test, verify

```bash
bash B20SS/verify.sh            # icd, cpp (configure + build + ctest + cppcheck), py (186 tests), load
bash B20SS/verify.sh icd py     # any subset, in order
```

By hand, from `B20SS/ofp/` in a UCRT64 shell or on Linux: `cmake --preset <preset>`,
`cmake --build --preset <preset>`, `ctest --preset <preset>`, `cmake --build build/<preset>
--target cppcheck`, `cmake --preset coverage && cmake --build --preset coverage`. The
Python suite by hand needs the `FMOFP_*` environment variables pointing at the binaries
(`verify.sh` shows which; `fmofp_bus/tests/_support.py` documents them); one test module:
`py -m unittest fmofp_bus.tests.test_mission_e2e` from `B20SS/` with those variables set.
`py icd/tools/icd_tool.py check|render|gen-py|gen-cpp` from `B20SS/`.

### 5.5 Running the aircraft

From `B20SS/`: `py -m fmofp_demo` starts the plant, every C++ partition, the six display
windows and the bus controller, prints one dashboard line per second and drives the scripted
events; Ctrl-C stops everything. Options: `--seconds S`, `--headless`, `--no-displays`,
`--no-events`, `--hold heading=0,speed=450`, `--seed N`, `--port-base N`, `--build DIR`.
Logs in `B20SS/logs/demo/` (ignored by git). `B20SS/ofp/README.md` has the per-process
command lines, `fmofp_bc --schedule --rt ...` prints the schedule and its load, and
`fmofp_bc --report` the per-terminal statistics.

### 5.6 Where things are

`B20SS/icd/` the ICD (XML source of truth, `README.md` conventions and decisions, generated
`ICD_TABLES.md`, `tools/icd_tool.py`). `B20SS/ofp/` the C++ (`bus/`, `bc/`, `osal/`,
`partitions/<name>/`, `tests/`, `CLAUDE.md` rules, `README.md`). `B20SS/fmofp_bus/` the
Python bus library and all port tests (`tests/`). `B20SS/fmofp_plant/` the plant.
`B20SS/fmofp_display/` the display terminals and feeds. `B20SS/fmofp_demo/` the launcher.
`B20SS/FMOFP/` the frozen Python application (`PLANNING.md` beside it describes its
architecture; its own tests are Alee's). `B20SS/verify.sh` the gate.

---

## 6. Gotchas and lessons (add yours; keep each to a line or two)

Bus and C++
- A `BlockPublisher` without `init(storage, capacity)` publishes a descriptor of zeros and
  no error (cost 25 minutes once).
- `fmofp_bc` rejects `--port-base` above 65000; every e2e test owns a distinct base
  (grep `PORT_BASE` in `fmofp_bus/tests` before choosing one).
- A bus-controller run with Python terminals needs `--timeout-ms 20` (40 for the busiest
  tests under the full suite): a Python plant answers several polls per frame.
- A 1 Hz TERM_STATUS needs a window of at least 50 frames in a test, a 5 Hz message at
  least 10; count frames from the MC17 syncs on the monitor.
- The gate tests' stub terminals must not use an RT that has messages now (RT 2 became
  comms: the stubs moved to RT 4).
- The load model: the fourteen-terminal aircraft sits at 6908 us; a 25 Hz block costs
  roughly a quarter of a frame per segment; before adding a 50 Hz message check the line.
- A custom CMake target that runs ctest must `add_dependencies` on the test executables;
  run test executables from the UCRT64 shell or with its `bin` on PATH (DLLs).
- The radar display's particle animation in the same process as a bus terminal starved the
  terminal (overruns): display terminals run in a helper process (`TerminalProcess`) and
  `cancel_join_thread()` their queue so a killed window leaves no orphan.
- Track tables age out after 15 frames without a report: tests that step many frames must
  re-feed the traffic each frame.

Python tests and displays
- Oracle tools print `ok` for `*_clear` commands: consume every answer in the order asked.
- The fake terminal of the feed tests stores unquantised values; a 1/16-LSB field compares
  with the quantised value, not the float you stored.
- Demo events must be in time order (a test checks); event sequence numbers start at 201 so
  the radar window's own numbering (from 1) never collides.
- The TSD paints every pixel of its panels: compare pictures, not lit-pixel counts.
- Offscreen Qt renders glyphs as boxes; layout and colours are still testable. Two display
  processes sharing the FMOFP log file name print a harmless `WinError 32`.
- `assertAlmostEqual(a, b, "msg")`: the third positional is `places`; use `msg=`.
- `install_system_sources()` must run before `mfd.py` is imported; the shared test process
  needs `uninstall_system_sources()` in a `finally`.
- The plant's airspeed is a random walk that ignores thrust (test aircraft, not an engine
  model): the autothrottle winds to full power in the demo by design.

Working on this machine (Kevin's)
- `.claude/CLAUDE.md` in Kevin's clone is a private pointer to this file; the root file is
  the shared one. Do not create a second working file.
- MSYS2's `bin` holds its own `python.exe` (3.14, no PyQt6): never put it above the
  python.org install on PATH; `py` is unaffected.

---

## 7. Unit log (append-only; newest at the bottom)

Format for new entries: `### <date> - unit N <name> - <commit> - by <name's agent>`, then
*What*, *How*, *Warnings / lessons*, *Files*, *Tests and numbers*. Entries for the units
done before this file existed are condensed from the private session notes.

### 2026-10-10 - units 1, 2, 3 bus core, ICD generators, Python bus library - f61c569..8755ed3, 8490842
- *What*: the simulated bus in C++ (`ofp/bus`: words, command and status words, framing,
  `RemoteTerminal` with per-SA buffers, status latches and mode codes 2/3/6/7/8/16/17/18/19;
  `ofp/osal`: absolute-deadline clock and UDP for win32/posix; `ofp/bc`: `Schedule` with
  phase spreading and the load estimate, `BusController` = MC17 sync, scheduled slots,
  aperiodic queue, per-RT stats, latest-data table, monitor copy; `fmofp_bc`, `fmofp_rt_stub`),
  the ICD (`icd/fmofp_icd.xml`, `icd_tool.py check|render|gen-cpp|gen-py`, generated
  enums/messages/blocks headers and modules with round-trip tests), the Python twin
  (`fmofp_bus`: status, word, transport, remote_terminal, rt_process, monitor).
- *How*: the ICD was drafted from `rtAddressConfig.xml` + `message_schemas.py` (89 legacy
  types mapped), Alee answered ten questions (UDP 51000, big-endian, UCRT64, BC own process,
  one process per subsystem and per display, freeze the Python, true heading), then Kevin
  took the ICD over. Gate: 150 frames with two stubs, 0 overruns, worst frame 285 us.
- *Warnings*: Windows UDP needs `SIO_UDP_CONNRESET` off or a dead peer's ICMP breaks
  `recvfrom`; the BC's latest-data table is 80 KB (keep the controller static); Python
  `encode_scaled` uses llround semantics, not `round()`; the old Python codec cannot encode
  radar modes (documented in the ICD README section 10; Alee: leave it).

### 2026-10-10 - unit 4 FCS - d5b8061 (ICD v1.1)
- *What*: `ofp/partitions/fcs`: GCAS (`_evaluate` decision for decision: PULL UP vs <= -4000
  and alt < 2000, SINK RATE, TERRAIN clearance < 500, ALT LOW < 200) and the performance
  monitor's envelope (g 9, AoA 25, roll 60, pitch 45, VS 6000; fuel efficiency), the mode and
  surface commands with fmsControl's clamps, state INITIALIZING/NORMAL/DEGRADED on stale
  sensors (3 frames). The Python FCC itself was a random-walk plant, so it became the plant
  (`fmofp_plant`, RT 27, `PLANT_SENSORS` 50 Hz relayed as `FCS_SENSORS`).
- *How*: oracle test `test_fcs_e2e.py` runs the frozen `GCAS()._evaluate` beside the real
  processes over the real bus (six scenarios).
- *Warnings*: the Python GCAS's terrain clearance import named a function that does not
  exist, so its TERRAIN alert never fired (the port gets clearance from the TFR, unit 8);
  outputs after init are all-zero FAILURE_WARNING, NO_COMPUTED_DATA once stepped without data.

### 2026-10-11 - unit 5 nav - (ICD v1.2 with unit 6)
- *What*: `ofp/partitions/nav`: the 12-satellite constellation, pseudoranges, WLS solver
  (max 8 iterations, tolerance 0.01 m, weights sin^2(el) + 0.01, 4x4 elimination with partial
  pivot), Bowring ECEF to WGS84, the per-axis scalar Kalman fusion, dead reckoning; `PLANT_GPS`
  (1 Hz, epoch + visible mask + 12 pseudoranges as 32-bit words), `NAV_SOLUTION` 10 Hz.
- *How*: `fmofp_nav_tool` (stdin) lets `test_nav_oracle.py` compare WLS/Bowring/fusion/
  visibility with the frozen Python; `test_nav_e2e.py` over the bus.
- *Warnings*: from the Python's INS start point (Tinker AFB) exactly four coplanar satellites
  are visible and the Python's solver is singular unless bootstrapped from the truth; the
  plant home is therefore 0 N 0 E (where the Python's fixes were really made). A Bancroft
  closed form seeds the WLS. The Python re-noised the pseudoranges every WLS iteration (the
  plant draws the noise once per epoch); the INS was noise with no consumer (not carried).
  The plant refreshes the GPS message every walk step so a fix polled late in the second is
  current.

### 2026-10-11 - unit 6 FMS - 5a42ffe (ICD v1.2)
- *What*: `ofp/partitions/fms`: flight-mode profiles (NORMAL/COMBAT/STEALTH/TRAINING/
  EMERGENCY limits), `update_attitude` envelope limiting with its warnings, thrust and gear
  (`set_thrust`: afterburner can never light through it, kept), autopilot targets, maneuvers
  (BARREL_ROLL not implemented, as in the Python), Mach, energy state, energy
  maneuverability; messages FMS_FLIGHT_DATA 10 Hz (replaces the FMS_DATA block), FMS_STATUS,
  FMS_ATTITUDE_STATUS, FMS_NAV_UPDATE_STATUS, FMS_CONTROL_CMD/STATUS, FMS_ENERGY, inputs by
  relay (FCS attitude, sensors, nav solution). `fmofp_bc --script FILE` ("frame rt sa words")
  queues aperiodic commands for tests.
- *How*: `test_fms_e2e.py`: eleven scripted commands; the oracle is the frozen `FMSControl`
  in-process with its attitude dict reset to the measured zeros before each command.
- *Warnings*: the Python's dead-reckoning rates were always zero (a ground-speed key the FMS
  never wrote); the port uses airspeed as ground speed (no wind model yet, backlog). On Linux
  the monitor socket overflowed when a test drained it only after the run: `Monitor.
  read_until()` reads while the BC runs.

### 2026-10-11 - unit 7 weather radar - 13cbcb9, 87ee524 (ICD v1.3)
- *What*: `ofp/partitions/radar` (weather_model, weather_products, weather_radar): the
  360 x el x 200 reflectivity volume, cells and stratiform areas, clutter, VIL and
  precipitation products with `round_to_sigfigs(3)`, storm-cell analysis, wind shear and
  turbulence processors; scans of 60 frames; products as the first block transfer
  (`RDR_WX_PRODUCTS`, 548 words, header + six sections). The weather world moved to the plant
  (`fmofp_plant/weather.py`, `PLANT_WEATHER`). Block transport in both libraries
  (`ofp/bus/block.*`, `fmofp_bus/block.py`).
- *How*: `fmofp_radar_tool` + `test_weather_oracle.py` compare every stage with the frozen
  generators and processors; texture noise is a hash (seed, az, el, r) so the oracle can
  turn it off.
- *Warnings*: the Python's storm-cell tracker deletes every new cell in the call that created
  it (no cell ever reached a display); the precipitation generator does not round azimuths
  before radians but VIL does (both mirrored); scipy's `uniform_filter1d` running sum has a
  specific order (`tmp += line[i+2] - line[i-1]` with reflect padding); numpy's label order
  is raster order of the first pixel; the 4 MB volume must be static. Shifts of promoted
  narrow integers trip `-Wsign-conversion` only under UBSan (cast first).

### 2026-10-25 - unit 8 terrain-following radar - af838a4, 019c8f8 (ICD v1.4)
- *What*: TFR in the same radar process: clearance levels CLEAR/LOW/CAUTION/PULL_UP (600/300/
  150 m) with a latching master, the path optimiser with the set clearance (30-5000 ft),
  terrain warnings; the terrain moved to the plant (`fmofp_plant/terrain.py`: hills =
  1000 + 500 sin(n/1000) cos(e/2500) + 200 sin(n/300 + e/700) plus two ripples; `PLANT_TERRAIN`
  block 100 samples over 10 km ahead, 10 Hz); block relays and receive blocks in the ICD;
  `FCS_TERRAIN` gives GCAS its clearance (the Python never could).
- *Warnings*: the Python assessed every profile at a fixed 1000 m altitude and never called
  `update_speed`; `PLANT_TERRAIN` later went to 25 Hz with a 5 Hz profile when the load
  needed it (rule: n segments need 2n frames at 25 Hz). BC runs with Python terminals need
  `--timeout-ms 20`.

### 2026-10-25 - unit 21 radar display + PFD on the bus - 5abb03a, f7c9b41, 1838cc4 (ICD v1.5)
- *What*: `fmofp_display` (`terminal.py` DisplayTerminal, `radar_feed.py`, `flight_feed.py`,
  `app.py`): the unchanged `WeatherRadarDisplay`, `TFRRadarDisplay` and
  `PrimaryFlightDisplay` fed from the bus; the radar display is the radars' control panel
  (its commands on SA 20-24; the MFD commands nothing); `cmd_seq` 0 = no command; the BC
  retries a silent transaction once.
- *Warnings*: positions are true nm at the radar's range (the Python drew a fixed 40 nm
  frame); the TFR display raises on a None clearance (feed passes 0.0/UNKNOWN); Qt tests on
  CI need the apt packages of section 5.3.

### 2026-10-25 - unit 22 MFD on the bus - 3848ef6 (ICD v1.6)
- *What*: `systems_feed.py` stand-in modules (PEP 562 `__getattr__`) for every getter
  `mfd.py` imports inside its pages; the embedded radar displays through the plain factory;
  the radar product blocks moved to 25 Hz with least-loaded phase assignment (the 50 Hz copies
  had hit 10352 us).
- *Warnings*: `install_system_sources()` before importing `mfd.py`; the MFD's ENGINE page is
  unreachable in the Python; new schedule phases exposed timing assumptions in two tests
  (fixed properly: the plant's GPS refresh, the radar e2e's first scan with weather).

### 2026-10-25 - unit 23 broadcast relays - 181b39e (ICD v1.7)
- *What*: RT 31 broadcasts with `listeners`, `accept_broadcast` in both terminal libraries
  (before it, any terminal stored any broadcast on any SA), `check_broadcasts` collision
  rule, the BC schedules a broadcast relay when any listener is present; the per-display
  copies removed. Load 8136 -> 7056 us with eight terminals.

### 2026-10-25 - unit 9 engine management - fe8cdc9 (ICD v1.8)
- *What*: `ofp/partitions/engine`: `ThrustManager` (PI autothrottle kp .5 ki .05, asymmetry),
  `FuelSystem` (tanks ML/MR/C, burn from measured flow, the Center top-up policy, transfers,
  low/critical/leak), `EngineManagement` (FMS lever drives both engines, ENG_CMD); the ECU's
  engine model (N1/N2/EGT/flow/oil/vibration formulas with its noise order) moved to the
  plant (`fmofp_plant/engine.py`, `PLANT_ENGINE`, reads `PLANT_THRUST_CMD`). First subsystems
  that listen to broadcasts (engine hears FMS flight data, FCS hears engine status).
- *Warnings*: the Python had three unconnected thrust numbers and tanks that never moved;
  `replaces=` in the ICD must name a legacy type that exists; a FuelSystem test that drains a
  main must keep the Center at or below 5000 kg or the policy refills it.

### 2026-11-01 - demo launcher and display helper processes - e9336ec, 54b4656
- *What*: `py -m fmofp_demo` (Kevin's "see it work" command): starts everything, dashboard
  line per second, scripted events, logs in `logs/demo/`; `TerminalProcess` runs each display
  terminal in a helper process (the weather widget's particle animation had starved the bus
  thread: 416 overruns), `cancel_join_thread()` so a killed window leaves no orphan helper.
- *Numbers*: 180 s run for Kevin: 9000 frames, 0 overruns, worst 6.9 ms, 9/9 answering.

### 2026-11-01 - unit 24 EICAS on the bus - fc50fdd (ICD v1.9)
- *What*: RT 14 listens to seven broadcasts; `SystemsFeed` answers the ten getters
  `eicas.py` polls; `RDR_WX_SCAN.max_shear` added; engine readings through
  `set_engine_readings`.
- *Warnings*: `_compute_alerts(None)` raises (the profile power is always given); the alert
  list grades the thrust model's EGT, not the measured one (kept, asserted). Hydraulics C,
  electrical and ECS rows stay UNKNOWN until units 16 and 20.

### 2026-11-01 - unit 10 targeting radar + track fusion - 226f3c3 (ICD v1.10)
- *What*: `target_processing` (signature analysis, stealth Bayes, threat rule),
  `targeting_radar` (12-track table, gate 1000 m, drop after 15 frames, lock), `track_fusion`
  (epochs every 25 frames, greedy clusters within 500 m, confidence 1 - prod(1 - w)); the air
  traffic moved to the plant (`fmofp_plant/traffic.py`, `PLANT_TRAFFIC`). `PLANT_TERRAIN` to
  25 Hz for the load.
- *Warnings*: a 4-segment block polled at 25 Hz needs a source at 5 Hz or slower; libm-fragile
  tie cases between two reference RCS values are not testable.

### 2026-11-01 - unit 25 TSD on the bus - 26a14f6, bb08383 (ICD v1.11)
- *What*: RT 16 with the fused tracks block, FMS data and energy; the MFD targeting page and
  the radar window's targeting controls. The feed maps the radar's class names into the
  display's symbol vocabulary (the Python's never matched: every target was a '?').

### 2026-11-01 - unit 11 AEWC radar - a3a9521, 680f358, 2a59729 (ICD v1.12)
- *What*: AEWC radar in the radar process (radar equation kept as written: dBW/dBm mix,
  SNR 0 beyond about 40 km; sectors from EAST counter-clockwise; ECM assessment; priorities);
  reports join the fusion (weight 0.7). One display-class fix (`aewc_radar_display.py`
  `_draw_target` arity: every conventional target raised). Load model: a relay costs one
  RT-to-RT transfer per consumer (11 terminals 9712 -> 6368 us).

### 2026-11-01 - unit 12 SAR radar - 499272a (ICD v1.13)
- *What*: strip imaging 2 km right of track at 64 x 64 on the bus (2075-word block every
  150 frames), the change detector step for step (log floor, 11 x 11 box filter with scipy's
  order, mean + 3 sigma, erode/dilate, 4-connected labels, regions); the ground scene moved
  to the plant (`fmofp_plant/ground.py`, `PLANT_GROUND`); the MFD SAR page decodes a PGM.
- *Warnings*: the Python's CFAR has no guard cells, so any compact change masks itself (the
  moving ship is never reported) and scipy's running-sum residue reports unchanged ground as
  CHANGE: both reproduced exactly and documented (a future intern ticket). Images every 3 s
  exceed the display's staleness window: the feed keeps the last image.

### 2026-11-02 - unit 26 SMS on the bus - 8e1eb62 (ICD v1.14)
- *What*: RT 17 listens to `BCAST_FMS_STATUS` only; the stations, release modes and signature
  are the display's own (no armament system exists in the Python: a stores computer is
  backlog). Quirk kept: the station table paints with the last icon's brush.
- *Decision*: the HUD (unit 27) is an empty placeholder in the Python; Kevin struck it.

### 2026-11-02 - one consumer, its own relay - ec755a9 (ICD v1.15)
- *What*: the broadcast map was full; single-consumer streams (targeting, AEWC and SAR
  pictures to the MFD; energy to the TSD) became the consumers' own relays, freeing seven SAs.

### 2026-11-02 - units 13 + 14 mission computer + communications - 051e57d (ICD v1.16)
- *What*: `ofp/partitions/mission` (`RouteManager`, `Targeting`, `OrderOfBattle`,
  `MissionComputer`: route progress, ETEs, target insertion, threat deviation, the target
  status machine with quadratic decay, the plan block every 75-150 frames) and
  `ofp/partitions/comms` (`CommsComputer`: radio active above 20 % field, satcom connected
  when the plant's satellite is in view, data link active iff connected, COM_CMD clamps); the
  RF environment moved to the plant (`fmofp_plant/rf.py`, `PLANT_RF`). MFD comms and mission
  pages, TSD route and contacts, demo events (14 terminals).
- *Warnings*: the Python's second route (`MissionPlanningSystem`) was write-only; its
  position guard against (0, 0) and a misspelled ground-speed key made every ETE 450 kt;
  `insert_target` on an empty route deadlocked the service (the port appends). Test slips
  worth remembering: ids keep counting across `clear()`; a LOST target keeps decaying; unit
  ids count across affiliations.
- *CI*: run 41569356013 all ten jobs green.

### 2026-11-04 - upstream merge d82173f; this file and verify.sh - by Kevin's agent
- *What*: merged upstream/main `4c25f52` (six commits of 2026-11-02: lint sweep, logging,
  dead-code removal; 253 files, conflict-free; every mirrored module changed only by removed
  unused imports and plain strings; full suite 186 OK on the merged tree). Created this
  shared working file for the two-person workflow, `B20SS/verify.sh` as the one gate, and
  retired the private `.claude/CLAUDE.md` to a pointer (its notes are archived on Kevin's
  machine).
- *Numbers*: meter unchanged, 21 / 29.5 = 71 %.

---

## 8. Decisions (append-only: date, who, what, why)

- 2026-10-03, Kevin: port the flight software to C++ (restricted C++17), per Alee's two
  documents; displays, scenario engine, CLI, storage, installer and tests stay Python; the
  bus is the only link. The Python-era SSTR training tickets are legacy; new tickets will be
  written against the C++.
- 2026-10-10, Alee (via Kevin): freeze the Python at `0ab9236` until the conversion is done;
  bus controller as its own process; one process per subsystem and per display; UDP on
  127.0.0.1 from port 51000; UCRT64 toolchain on the intern machines; true heading; leave the
  old Python codec's bugs.
- 2026-10-10, Kevin: no more approvals through Alee; the ICD and every design choice are ours.
  Commit and push each milestone to `origin port/cpp` without asking ("yes and yes, do your
  thing").
- 2026-10-11, Kevin: interns must debug C++, so flight logic goes to C++ and displays stay
  Python ("skip the PFD, do the FMS/nav in C++").
- 2026-10-25, Kevin: keep pulling upstream/main into `port/cpp` while it merges cleanly and
  does not interfere with the mirrored logic; otherwise do not pull and say so.
- 2026-10-25, Kevin: every summary states the conversion percentage and the time to 100 %.
- 2026-11-02, Kevin: skip the HUD (unit 27 struck; total weight 29.5); do mission and comms.
- 2026-11-04, Kevin: two people alternate on the fork, each with Claude; the shared working
  file is this `CLAUDE.md` at the repository root; every session pulls first and reads it,
  every turn that changes anything updates it and pushes; `B20SS/verify.sh` is the gate.
