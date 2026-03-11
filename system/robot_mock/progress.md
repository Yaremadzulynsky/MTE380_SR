Original prompt: "There is a new folder under system called robot_mock. I need you to create a 2d simulation of the robot that takes commands from the system via the mock control communication module. this will effectivly let me test out my logic and system in a  virtual world while I give it simulated inputs."

Notes
- 2D sim runs in the browser (canvas) and polls control-communication for the latest `/control` command and `/state` state-machine state.
- Simulated computer-vision-style inputs are POSTed directly to state-machine `/inputs` via the robot-mock backend.
- `/api/system` is now cached/throttled to avoid hammering upstream services and spamming logs; frontend poll interval is reduced.

TODO
- Add optional obstacles/walls and collision response.
- Add a “scenario” menu (auto-run, reset, randomize).
- If you want full closed-loop autonomy, add a controller module that turns FSM state into `/control` commands.
- Playwright-based UI automation (skill loop) is blocked unless `playwright` is available in the environment.
- Consider setting `ROBOT_MOCK_API_TOKEN` (and optionally `QUIET_ACCESS_LOG=1`) if the port is exposed beyond localhost.

Update (line error signal + controller)
- Changed `public/main.js` line observation to report front-point cross-track error vector:
  - `line.vector` is now the robot-frame vector from robot-front to closest point on line (perpendicular by projection).
  - Magnitude now directly represents distance from robot front to line in world units.
- Updated `state_machine/sm_state_machine.py` line-follow behavior to treat `inputs.red_line` as an error signal (not direction):
  - Added proportional correction parameters: `LINE_ERROR_GAIN`, `LINE_ERROR_DEADBAND`, `LINE_FORWARD_BIAS`.
  - Added `_line_follow_command(...)` and applied it in `SEARCHING` and `ALIGN_FOR_RETRIEVE`.

Verification
- `node --check system/robot_mock/public/main.js` passes.
- `python3 -m py_compile system/state_machine/sm_state_machine.py` passes.
- Playwright skill client invocation is currently blocked by local Node module mode mismatch (`Cannot use import statement outside a module`).

Update (vector-only SM payload + UI toggle)
- Updated `system/robot_mock/public/main.js` so the simulator now sends only four robot-front-relative vectors to state machine `/inputs`:
  - `safe_zone: {x, y}`
  - `target: {x, y}`
  - `danger_zone: {x, y}`
  - `home: {x, y}`
- Removed all other mock-generated state-machine fields from outbound payload (alignment flags, line payload, pickup/place pulses, at-home flags, etc.).
- Added a HUD toggle to show/hide these vectors on the canvas:
  - `system/robot_mock/public/index.html`: added `#vectors-toggle` checkbox.
  - `system/robot_mock/public/main.js`: draws colored arrows from robot front to the four vector endpoints and toggles visibility.

Verification
- `node --check system/robot_mock/public/main.js` passes.
- `python3 -m py_compile system/robot_mock/app.py` passes.
- Playwright skill client attempt failed due to missing dependency in environment:
  - `Error [ERR_MODULE_NOT_FOUND]: Cannot find package 'playwright' imported from .../web_game_playwright_client.js`

Update (line error vector forwarded + typed SM input)
- Added `line_error` vector to robot-mock state-machine payload (`system/robot_mock/public/main.js`).
  - Source is the existing perpendicular robot-front-to-line vector (`lineObs.vector`).
- Added `line_error` to on-screen vector overlays when `Show vectors` is enabled.
- Added explicit typed field to state-machine Inputs model:
  - `system/state_machine/sm_models.py`: `Inputs.line_error: Vector`
- Updated input parsing:
  - `system/state_machine/sm_input_buffer.py` now parses `line_error` from payload.
  - For compatibility with existing logic, `red_line` is set to `line_error` when present; otherwise it keeps legacy `line/home` behavior.

Verification
- `node --check system/robot_mock/public/main.js` passes.
- `python3 -m py_compile system/state_machine/sm_models.py system/state_machine/sm_input_buffer.py` passes.
- Playwright skill client still blocked by local Node module mode issue:
  - `SyntaxError: Cannot use import statement outside a module` for `web_game_playwright_client.js`.
- Adjusted red-line/line-error measurement origin in `system/robot_mock/public/main.js`:
  - Now measured from one robot length in front of the robot front (probe point), per request.
  - Updated vector rendering origin to the same probe point so displayed vectors match what is sent.
- Added robot kinematics to simulator input payload:
  - `heading_rad` and `speed` now sent with `/api/state-machine/inputs` from `system/robot_mock/public/main.js`.
- Added explicit typed fields to state-machine `Inputs`:
  - `heading_rad: float`
  - `speed: float`
- Updated `system/state_machine/sm_input_buffer.py` parsing to validate and populate these fields.
- Vector payload enrichment:
  - Robot mock now sends each vector with `x`, `y`, `heading`, and `magnitude`.
  - Applied to `safe_zone`, `target`, `danger_zone`, `home`, and `line_error`.
- State-machine vector model now exposes derived polar values:
  - `Vector.heading` (rad)
  - `Vector.magnitude`
  - `Vector.to_dict()` includes both.
  - `Vector.from_dict()` now supports polar-only input (`heading` + `magnitude`) in addition to x/y.
- Added polar command path in state machine:
  - New `_send_control_polar(angle_rad, speed, context)` converts angle/speed to control `(x, y, speed)`.
  - `State.TESTING` now commands using `inputs.red_line.heading` and `inputs.red_line.magnitude` instead of direct `inputs.red_line.x`.
- Added pen tool for arbitrary red-line drawing in simulator:
  - New HUD controls: `Pen: On/Off` and `Clear Line`.
  - Keyboard shortcut `l` toggles pen mode.
  - In pen mode, click-drag on canvas to draw a polyline path.
  - `Clear Line` resets path to default `home -> target` line.
- Line-follow observation now uses nearest segment of the drawn path instead of a single home-target segment.
- Rendering updated to draw the full polyline path; path also exposed in `render_game_to_text()`.

Verification
- `node --check system/robot_mock/public/main.js` passes.
- Playwright skill client still blocked by environment module mode issue (`Cannot use import statement outside a module`).
- Improved line-follow command logic in `state_machine/sm_state_machine.py` (`State.TESTING`):
  - Replaced fixed `heading +/- 0.5` behavior with smooth lookahead steering:
    - `angle_rad = atan2(red_line.x, red_line.y + lookahead)`
  - Added speed scheduling based on both line distance and steering demand.
  - Keeps command logic in state machine and still uses `_send_control_polar(...)`.
- Fixed `FSM unknown` root cause by publishing state-machine state to control-communication during normal runtime.
- `system/state_machine/app.py` now syncs state:
  - immediately on startup,
  - whenever a transition occurs,
  - periodically via `STATE_SYNC_INTERVAL` heartbeat (default 1.0s),
  - once on end-state exit.
- Fixed always-true detection flags from sim vectors:
  - Payload vectors now include explicit `detected` booleans.
  - Added range-gated detection in robot mock for `target/safe/danger/home`.
  - Added explicit `line_error.detected` from line observation.
- This prevents `lego_detected` from being true when far from target.
- Added explicit robot vision area in simulator:
  - New forward-facing square FOV (`visionSquareSize`) rendered on canvas.
  - POI vectors (`target`, `safe_zone`, `danger_zone`, `home`) are now marked detected only when inside that square in robot frame.
  - When POI is outside FOV, payload now sends `detected=false` and zero vector for that POI (prevents leaking out-of-view target direction).
  - `line_error` behavior unchanged.
- Added explicit `home` vector field to state-machine inputs:
  - `Inputs.home: Vector` in `state_machine/sm_models.py`.
  - Parsed/populated from payload in `state_machine/sm_input_buffer.py`.
  - Existing `red_line` fallback-from-home behavior kept for backward compatibility.
- Added course save/load in robot mock UI:
  - `Save Course` exports JSON with current path + POIs.
  - `Load Course` imports JSON and applies path + POIs to current sim.
  - Added input validation/clamping for loaded points and danger radius.
  - Uses browser download/upload (no backend changes required).
- Added explicit `RETRIEVED` 180-degree turn behavior in state machine:
  - Latches target heading = current heading + pi on enter `RETRIEVED`.
  - Sends turn command from state machine until heading error < tolerance.
  - Sends stop, then transitions to `TRANSPORTING` with label `R180`.
  - Tunables: `RETRIEVED_TURN_SPEED` (default 0.35), `RETRIEVED_TURN_TOL_RAD` (default 0.12).
- Refactored red-line following in `state_machine/sm_state_machine.py`:
  - Removed duplicated lookahead steering blocks from active state branches.
  - Added single PID-based `follow_line(...)` function.
  - Updated states to call `follow_line(inputs.red_line, now=..., context=...)`.
  - Added PID tunables via env vars (`LINE_PID_*`).
  - PID state resets on line loss and state transitions.
- Added live line-follow PID tuning path end-to-end:
  - `control_communication`: new `GET/POST /line-follow-pid` with validation + in-memory settings.
  - `state_machine`: periodically pulls `/line-follow-pid` and applies gains live (`LINE_PID_SYNC_INTERVAL`).
  - `control_screen`: new `/api/line-follow-pid` proxy + range config.
  - `pid.html` + `app.js`: new "Line Follow PID (Live)" fields with debounced live updates on input.
- Added dedicated line-follow secondary speed cap: `follow_max_speed`.
  - Enforced in state machine after primary line-follow speed shaping/max clamp.
  - Exposed in control-communication `/line-follow-pid` and control-screen live tuning UI.
  - Env fallback: `LINE_PID_FOLLOW_MAX_SPEED` (default 1.0).
- Fixed sim `Restart FSM` button behavior:
  - Button now POSTs `/api/state-machine/restart` instead of only refreshing view.
  - On success/failure, it refreshes system state so HUD updates immediately.
  - Button text/title now correctly indicate restart/reset behavior.
- Fixed sim pause behavior being overridden by auto-resume:
  - Added `manualPaused` latch.
  - Auto-resume from non-zero control now only triggers when `manualPaused` is false.
  - `Pause`/`p` sets `manualPaused=true`; `Start` clears it.
- Updated default line-follow PID values to requested set:
  - kp=0.2, ki=0.04, kd=4.92, i_max=20, out_max=20,
    base_speed=1, min_speed=0.18, max_speed=1,
    turn_slowdown=5, error_slowdown=0.14, deadband=0.01.
- `follow_max_speed` default kept at 1.
- Applied both at control-communication defaults and state-machine fallback env defaults.

Update (dynamic camera FOV controls)
- Added runtime camera FOV controls in robot mock HUD:
  - `Vision Depth`: how far ahead the robot can see.
  - `Vision Width`: how wide the visible area is.
  - Files: `system/robot_mock/public/index.html`, `system/robot_mock/public/style.css`, `system/robot_mock/public/main.js`.
- Replaced fixed square-size FOV with dynamic width/depth parameters:
  - Detection gate (`inVisionSquare`) now uses `visionDepth` and `visionWidth`.
  - Line-observation clipping region (`computeLineFollowObservationOnPath`) now uses same dynamic bounds.
  - FOV overlay rendering now draws a rectangle with independent depth and width.
- Exposed current FOV in `render_game_to_text()` output under `vision_fov`.

Verification
- `node --check system/robot_mock/public/main.js` passes.
- Playwright skill client invocation attempted and still blocked by environment module mode issue:
  - `SyntaxError: Cannot use import statement outside a module` for `web_game_playwright_client.js`.
- Fixed line vector leaking outside vision square:
  - `line_error` now also requires `inVisionSquare(...)` to be detected/sent/rendered.
  - Outside square, `line_error` payload is zero with `detected=false`.
- Fixed persistent Restart FSM failure:
  - State-machine worker loop no longer exits at `END`; it keeps running so reset can revive FSM behavior.
  - Restart button now polls `/api/system?fresh=1` after reset to avoid stale cached FSM state.
- Tightened line-error square gating to match visible square frame:
  - Converted probe-relative line vector to front-relative for `inVisionSquare(...)` check.
  - Cached draw vector now uses gated `linePayload` (zero when outside), eliminating stale visual arrows.
- Root-cause fix for persistent line-only false detection:
  - In `sm_input_buffer`, `red_line` previously fell back to `home` whenever `line_error.detected` was false.
  - Now: if `line_error` key is present in payload, `red_line` always equals `line_error` (including detected=false).
  - Legacy `line/home` fallback only applies when `line_error` is absent entirely.
- Reworked line detection/measurement to be camera-view-box based:
  - Path segments are clipped to the robot vision square in robot frame.
  - `line_error` uses the closest point on the *visible clipped segments* to the probe point.
  - If any clipped segment exists, `line_error.detected=true`; otherwise false.
  - This matches "only information visible in the camera view" behavior.

Update (FSM force-state override from control screen)
- Added explicit state-machine override APIs in `system/state_machine/sm_api.py`:
  - `GET /states` returns allowed FSM states and current state.
  - `POST /set-state` accepts `{ "state": "..." }`, validates against enum, clears input buffer, and forces state.
- Added deterministic state override reset logic in `system/state_machine/sm_state_machine.py`:
  - New `force_state(target_state)` resets runtime context and transient internals (`StateContext`, line PID accumulator, find-line scan state, retrieved-turn latch, command timestamp) before setting target state.
  - `reset()` now uses `force_state(State.SEARCHING)`.
- Added control-screen proxy endpoints in `system/control_screen/server.js`:
  - `GET /api/state-machine/states`
  - `POST /api/state-machine/set-state`
- Added UI controls on PID screen (`system/control_screen/public/pid.html` + `app.js` + `styles.css`):
  - New “State Machine Override” panel with state dropdown, refresh, and apply buttons.
  - UI shows success/error feedback and states are loaded from backend.

Verification
- `node --check system/control_screen/public/app.js` passes.
- `node --check system/control_screen/server.js` passes.
- `python3 -m py_compile system/state_machine/sm_api.py system/state_machine/sm_state_machine.py` passes.
- Playwright skill client remains blocked by local Node module mode mismatch:
  - `SyntaxError: Cannot use import statement outside a module` when running `web_game_playwright_client.js`.

TODO
- Consider removing or gating current hardcoded auto-advance test shortcuts in `sm_state_machine.step()` (e.g. states that set `pick_up_success/aligned_for_place/place_success` unconditionally), since those can make some manually-forced states transition immediately.

Update (line-follow PID sliders for live tuning)
- Converted Line Follow PID controls in `system/control_screen/public/pid.html` from number-only inputs to slider + numeric pairs for each field (`kp`, `ki`, `kd`, `i_max`, `out_max`, `base_speed`, `min_speed`, `max_speed`, `follow_max_speed`, `turn_slowdown`, `error_slowdown`, `deadband`).
- Added two-way slider/number synchronization in `system/control_screen/public/app.js`:
  - Dragging slider updates numeric value and triggers debounced live update.
  - Editing numeric value updates slider position and keeps backend payload behavior unchanged.
- Range min/max/step now applied to both slider and numeric input from backend bounds in `loadConfig` path.
- Added slider layout and responsive styles in `system/control_screen/public/styles.css`.

Verification
- `node --check system/control_screen/public/app.js` passes.
- `node --check system/control_screen/server.js` passes.

Update (line-follow speed range raised to 0..5)
- Increased line-follow speed bounds from `0..1` to `0..5` in state-machine runtime and validation:
  - `system/state_machine/sm_state_machine.py`
  - Added `MAX_CONTROL_SPEED = 5.0` and updated final line-follow control clamp + polar send clamp.
- Updated control-screen line-follow range defaults to expose `0..5` in UI hints/sliders:
  - `system/control_screen/server.js`
- Updated control-communication speed acceptance to `0..5` (for forwarded control commands) and line-follow bounds for consistency:
  - `system/control_communication/app.py`

Verification
- `python3 -m py_compile system/state_machine/sm_state_machine.py system/control_communication/app.py` passes.
- `node --check system/control_screen/server.js` passes.
- `node --check system/control_screen/public/app.js` passes.

Update (remove robot-mock reads from control-communication /control)
- Removed robot-mock backend dependency on `CONTROL_COMM_BASE_URL` and `CONTROL_COMM_CONTROL_PATH`:
  - `system/robot_mock/app.py`
  - `/api/system` now returns a local neutral control payload (`x=0, y=0, speed=0`) and only fetches state from state-machine (`/states`).
- Updated robot-mock compose env to remove `CONTROL_COMM_*` wiring:
  - `system/docker-compose.yaml` (`robot-mock` service).
- Updated robot-mock docs to reflect new behavior:
  - `system/robot_mock/README.md`.

Verification
- `python3 -m py_compile system/robot_mock/app.py` passes.
- `rg -n "CONTROL_COMM|control-communication|/control" system/robot_mock -S` confirms no runtime `/control` fetch path remains in robot-mock backend.

TODO
- If autonomous movement should still be driven by state-machine outputs without reading control-communication, add a state-machine endpoint for the currently desired command and point robot-mock to that.
- Updated robot-mock UI help text to remove control-communication driving instruction:
  - `system/robot_mock/public/index.html`
- Wired direct state-machine -> robot-mock simulator control mirroring:
  - Added `POST /api/sim-control` in `system/robot_mock/app.py` to accept `{x,y,speed}` and store latest simulated control.
  - `GET /api/system` now reports last posted simulator control command (source: `state_machine_posted_sim_control`).
  - Updated `system/state_machine/sm_control_comm.py` mirror post target from control-communication vector endpoint to robot-mock endpoint.
  - Added `ROBOT_MOCK_CONTROL_URL` config plumbed through `system/state_machine/sm_state_machine.py` and `system/docker-compose.yaml`.

Verification
- `python3 -m py_compile system/state_machine/sm_control_comm.py system/state_machine/sm_state_machine.py system/robot_mock/app.py` passes.

Update (simulator mirroring moved to control-communication)
- Moved simulator-forward responsibility to `control_communication` so every accepted `/vector` or `/control` command is forwarded to robot-mock regardless of hardware mode.
  - Added `ROBOT_MOCK_CONTROL_URL` env (default `http://localhost:8200/api/sim-control`) and `post_to_simulator(...)` in `system/control_communication/app.py`.
  - Forwarding happens on every valid command path via `process_vector_payload(...)`.
- Removed temporary direct state-machine -> robot-mock mirror from `system/state_machine/sm_control_comm.py`.
- Removed `ROBOT_MOCK_CONTROL_URL` wiring from state-machine; added it to control-communication service env in compose.

Verification
- `python3 -m py_compile system/control_communication/app.py system/state_machine/sm_control_comm.py system/state_machine/sm_state_machine.py system/robot_mock/app.py` passes.
- Fixed joystick non-reaction path when state-machine is in `REMOTE_CONTROL`:
  - `system/control_screen/server.js` sends joystick vectors to state-machine `/inputs` as `red_line`.
  - `system/state_machine/sm_state_machine.py` (`State.REMOTE_CONTROL`) now consumes `inputs.red_line` directly and emits control commands (stop on near-zero magnitude), instead of only polling `/control`.
  - This restores flow: control-screen -> state-machine -> control-communication -> robot-mock simulator mirror.

Verification
- `python3 -m py_compile system/state_machine/sm_state_machine.py system/control_communication/app.py system/robot_mock/app.py system/control_screen/server.js` passes.
- Added turn sensitivity control in robot-mock HUD:
  - New button `#turn-sensitivity-btn` in `system/robot_mock/public/index.html`.
  - Button cycles turn presets `Low/Medium/High` in `system/robot_mock/public/main.js`.
  - Presets update `CONFIG.turnAggression`, `CONFIG.maxTurnRate`, and `CONFIG.turnInPlaceRate` live.
  - Selection persists via `localStorage` key `robot_mock_turn_sensitivity_index`.
  - Added `turn_sensitivity` fields to `render_game_to_text()` output.
  - Added button width style in `system/robot_mock/public/style.css`.

Verification
- `node --check system/robot_mock/public/main.js` passes.
- Playwright skill client check attempted after turn-sensitivity update:
  - `node "$WEB_GAME_CLIENT" --url http://localhost:8200 ...` failed with `ERR_MODULE_NOT_FOUND: Cannot find package 'playwright'`.
  - Used MCP Playwright browser interactions as fallback; confirmed button text cycles `Turn: High -> Turn: Low -> Turn: Medium`.
- Updated robot-mock defaults:
  - `Send sim inputs` now defaults OFF.
  - `Show vectors` now defaults OFF.
  - Turn sensitivity default preset is now `Low`.
  - Added explicit `ui.inputsToggle.checked = state.sendInputs` on init to keep checkbox synced with runtime default.

Verification
- `node --check system/robot_mock/public/main.js` passes.
