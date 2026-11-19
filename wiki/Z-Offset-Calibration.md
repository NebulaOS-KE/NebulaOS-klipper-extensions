# Z-offset calibration and the load cell

How NebulaOS measures the Z offset with the KE's load cell, what the automatic load-cell check
does, and when the water-bottle calibration is needed. The page starts with what you see on the
printer and gets more technical further down. The last sections are for developers.

Code: `extras/nebulaos_calibration.py`, `nebulaos_probe_pair.py`, `nebulaos_z_offset_probe.py`,
`nebulaos_raw_contact.py`, `nebulaos_load_cell_mode.py`, `nebulaos_load_cell_calibration.py`,
`nebulaos_z_adjust.py`, at `6532091` (the `production` branch, shipped with NebulaOS 1.0.3).
Configuration: the firmware's `calibration.cfg` and `z_offset_probe.cfg`.

---

## In short

**The Z offset** is the height difference between where the CR Touch probe switches and where
the nozzle actually touches the bed. Klipper homes Z and measures the bed mesh with the CR Touch,
so it has to know this difference to put the nozzle at the right height for the first layer.

**NebulaOS measures it instead of asking you to.** There is no paper test. The KE has a
**load cell** - a small force sensor under the front-left corner of the bed. When the nozzle
touches the bed, the load cell feels it. NebulaOS lets the CR Touch measure the bed at one point,
then touches the same point with the nozzle, and the difference is the Z offset.

On the screen: **Calibration** → **Auto Calibration** (step 4 in the list). It takes about 7
minutes and needs no tools:

1. checks the load cell (the bed must be empty),
2. homes,
3. heats the bed and cleans the nozzle on the wipe pad,
4. brings the nozzle to its measuring temperature and lets it settle,
5. measures the Z offset (three touches),
6. measures the bed mesh,
7. saves both and restarts Klipper, then checks that the saved values are the ones measured.

At the end the screen shows the new Z offset next to the old one.

## What you need to do

- **Empty bed.** No print, no tools, no hand on it during the check and the touches - anything
  pressing on the bed changes what the load cell reads.
- **Clean nozzle and clean bed at the front-left corner.** The touches happen at X 20, Y 25,
  right above the load cell. Filament on the nozzle or dirt on the bed there makes the nozzle
  touch "early".
- **Do not touch the printer while it measures.** The load cell is sensitive enough to notice.

## Fine-tuning during a print

If the first layer is a little too high or too low, use the screen's Z offset buttons while
printing. NebulaOS saves that adjustment and applies it again after every restart
(`NEBULAOS_Z_ADJUST`, up to ±2 mm). It is kept separate from the measured Z offset, and it is
reset to 0 when a new calibration saves a new Z offset - because the adjustment was made against
the old one.

## Automatic or manual load cell

The load cell reports a raw number (ADC "counts"), not grams. There are two ways to use it,
chosen on the screen under **Calibration** → **Load Cell**:

| | **Automatic** (default) | **Manual** |
|---|---|---|
| What it needs | Nothing - a short check with the bed empty | A one-time calibration with a weighed 1.5 L water bottle |
| How a touch is detected | The raw signal changes by a fixed amount (a profile measured for the stock KE) | A force in grams (75 g), using your own bottle calibration |
| When to use it | Almost always | When the automatic check keeps failing, or your load cell behaves very differently from a stock KE |

**The automatic check** takes a few seconds with no motion: it listens to the load cell and checks
that it delivers samples at the right rate, is not stuck, not saturated, not too noisy and not
drifting. If it passes, the quiet reading is stored as the *unloaded reference*: the "zero" that
every later touch is compared with. Auto Calibration runs this check twice - once cold, before
anything moves, and once hot, just before the touches.

**The bottle calibration** (Manual) teaches the load cell how many counts one gram is:

1. empty bed - health check and zero,
2. stand the weighed bottle centred on the front-left bed screw, enter its weight (1500 g is
   pre-filled),
3. lift it and put it back - the reading must agree with the weight you entered,
4. remove it - the reading must return to zero,
5. optional: press with a finger to see the measured force,
6. save.

Saving a bottle calibration switches the mode to Manual. Choosing Automatic again switches back.

**Why a wrong weight is the dangerous mistake:** typing 150 instead of 1500 would make the printer
believe the cell is ten times more sensitive than it is, so a touch would press ten times harder
than intended. That is why the entered weight has a fixed allowed range. A bottle placed slightly
off the screw reads *low*, which makes touches stop *earlier* - the safe direction.

Neither mode changes the Z-offset result itself: the contact point is found by fitting the shape
of the force curve, which does not depend on the scale (see
[How the contact height is found](#how-the-contact-height-is-found)).

## When it stops with a message

| Message (short form) | Meaning | What to do |
|---|---|---|
| the automatic load cell check failed - … | The load cell was noisy, drifting, stuck or not delivering samples | Make sure nothing touches the bed or the printer, wait for the bed temperature to settle, try again. If it keeps failing: Calibration → Load Cell → Manual |
| the empty reading moved … counts from the reference | Something presses on the bed, or the reading drifted since the check | Clear the bed, run the check again |
| the load cell has not been calibrated yet | Manual mode without a bottle calibration | Do the bottle calibration, or switch to Automatic |
| measurement rejected - … | The touches disagreed too much, or the fit looked wrong | Clean the nozzle and the bed at the front-left corner, try again |
| No trigger on probe after full movement | The nozzle went down the whole allowed distance without the load cell triggering | Check that the bed is not resting on something, that the load cell cable is connected; see the details below |
| implies a … correction … exceeding max_offset_correction_mm | The new value differs from the old one by more than 2 mm | Something changed mechanically (nozzle, hotend, bed). A jump this large is refused on purpose: check what changed before trying again |

The messages name the reason for each rejected touch, so a report can say exactly what went wrong.

---

## How the measurement works

### The idea

Upstream Klipper's `PROBE_CALIBRATE` computes the probe's Z offset from two readings at the same
bed point: where the probe triggers, and where a human says the nozzle touches the paper. Written
out and simplified, the old Z offset cancels:

```
new z_offset = Z where the CR Touch triggers  -  Z where the nozzle touches
```

both measured in the same toolhead coordinates. NebulaOS does exactly that, with the load cell in
place of the human (`nebulaos_probe_pair.measure_probe_nozzle_pair()`):

1. Lift to `horizontal_move_z` (5 mm), move the **CR Touch** over the reference point and probe
   once: `raw_probe_trigger_z`.
2. Lift, move the **nozzle** over the same point.
3. Touch the bed with the nozzle `pro_cnt` times (default **3**), each touch with a fresh zero.
4. Keep the touches that pass the quality checks, average them.
5. `z_offset = raw_probe_trigger_z - average contact Z`.

Any `SET_GCODE_OFFSET` active at the time cancels the same way, because both readings are taken in
the same frame moments apart.

The reference point is `reference_x` / `reference_y` in `[nebulaos_calibration]`: **X 20, Y 25**,
the same point Creality's stock firmware uses for its own load-cell touch. It sits right above
the load cell, where a touch produces the clearest signal.

### One touch

`nebulaos_z_offset_probe.touch_probe()` runs on upstream Klipper's load-cell building blocks
(`hx71x` sensor, `load_cell`, `trigger_analog`, and `LCBestFit` from `load_cell_probe.py`); there
are no Klipper core patches.

1. **Arm.** Wait until all moves are done, then take a quiet window of samples and use it as the
   zero (the *tare*). Set the trigger threshold and a safety window in the MCU.
2. **Descend** at `contact_speed` (2 mm/s) as a Klipper probing move. The MCU stops the motors the
   moment the signal crosses the threshold - the host is not in that loop.
3. **Retract** `retract_dist` (2 mm) at `retract_speed` (10 mm/s) while still recording the load
   cell.
4. **Fit** the contact height (next section).

The MCU's stop is fast, but not instant: the nozzle has already pressed a little into the bed when
it stops. The fit corrects for that.

### How the contact height is found

The load cell's samples during the retract are fitted with upstream's piecewise least-squares
method (`LCBestFit`): the force is flat while the nozzle is free and rises in a straight line while
it presses. The point where the two lines meet is the real moment of contact, and its Z is looked
up from the motion history. That fitted Z - not the Z where the MCU stopped - is the contact
height.

The difference between the two (`fit_delta`) is how far past contact the nozzle went before it
stopped. It is a useful plausibility check: a large `fit_delta` means the fit found something
other than a clean contact.

The fit finds where the curve *bends*, so it does not depend on the force scale. That is why the
automatic mode (raw counts) and the manual mode (grams) give the same Z offset; on the development
printer they agreed within 0.006 mm.

### How deep the nozzle may go

The nozzle never searches without a floor. Two cases (`nebulaos_probe_pair.py`):

| Case | When | Floor for the nozzle |
|---|---|---|
| **Established** | A Z offset is already saved (not 0.000) | Predicted contact (`probe trigger Z - current z_offset`) minus `established_contact_margin_mm` (**1.0 mm**) |
| **Bootstrap** | No Z offset yet (exactly 0.000 = never calibrated) | `probe trigger Z - bootstrap_contact_envelope_mm` (**5.0 mm**) |

On top of that, `touch_probe()` never goes below the Z axis's own `position_min`, and the MCU stops
the motors when the force exceeds the safety limit, whatever the floor. These are independent
layers; the tightest one always wins.

If the nozzle reaches the floor without a trigger, the touch fails with "No trigger on probe after
full movement" and is not retried deeper. The 5 mm bootstrap envelope was raised from an earlier
3 mm floor after printers with silicone bed spacers needed about 3.5 mm before their load cell
triggered ([issue #9](https://github.com/NebulaOS-KE/NebulaOS/issues/9)).

### When a measurement is accepted

Each touch is checked:

- the fit produced finite numbers,
- the stop position is not below the floor,
- `|fit_delta|` is at most `max_abs_fit_delta` (**0.5 mm**).

Then the set of accepted touches:

- at least `min_accepted_samples` (**2** of 3) were accepted,
- their range (highest - lowest) is at most `max_repeatability_range` (**0.15 mm**),
- their standard deviation is at most `max_repeatability_stddev` (**0.06 mm**).

And for an established printer, the new value may not differ from the old one by more than
`max_offset_correction_mm` (**2 mm**). Only if everything passes is the result saved. A rejected
measurement never touches the configuration.

### Why the nozzle is measured at 150 °C

During Auto Calibration the hotend is not at printing temperature for the touches, but at
`z_offset_reference_temp` (**150 °C**), right after the nozzle clean. On the development printer,
touches after a long hot dwell at 230 °C read about 0.6 mm too low: plastic oozing from the nozzle
touched the bed first. At 150 °C, right after cleaning, the results were back in the normal range.
150 °C is also where the nozzle clean leaves the hotend, so this is normally no extra heating.

The bed is at `pid_bed_target` (**65 °C**) for the whole sequence, and there is an extra
`thermal_soak_seconds` (**15 s**) wait after both temperatures are reached.

### Why the new Z offset goes live only after the bed mesh

Auto Calibration homes Z with the *old* Z offset. If the new one went live before the bed mesh,
the mesh would be probed in a different frame than the homing and come out shifted by the
difference. So the new value is staged in the configuration, the mesh is measured, and only then
does the new Z offset go live. `SAVE_CONFIG` writes both, Klipper restarts, and after the restart
NebulaOS reads the saved values back and compares them with what it measured. The progress of
this sequence is kept in a journal file, so the verification after the restart knows what to
expect.

---

## For developers and tinkerers

### Commands

| Command | What it does |
|---|---|
| `NEBULAOS_AUTO_CALIBRATE` | The whole sequence above. Cancel with `NEBULAOS_CALIBRATION_CANCEL` (takes effect at the next stage) |
| `NEBULAOS_Z_OFFSET_CALIBRATE [X= Y=]` | Only the Z-offset measurement, saved with `SAVE_CONFIG` (Klipper restarts). Runs the automatic check first in Automatic mode |
| `NEBULAOS_LOAD_CELL_MODE [MODE=auto\|manual]` | Show or set the mode. No restart needed |
| `NEBULAOS_LOAD_CELL_AUTO_CHECK` | The automatic check on its own (no motion, empty bed) |
| `NEBULAOS_LOAD_CELL_CALIBRATE` | The guided bottle calibration (macro for `_NEBULAOS_LOAD_CELL_CALIBRATE`) |
| `NEBULAOS_LOAD_CELL_READ` | One reading of the load cell |
| `NEBULAOS_Z_ADJUST [DELTA=\|RESET=1\|APPLY=1]` | The saved Z offset adjustment |
| `NEBULAOS_RAW_CONTACT_DIAGNOSTIC`, `NEBULAOS_RAW_CONTACT_REFERENCE` | Diagnostics of the automatic contact |
| `PROBE_CALIBRATE` | Upstream's manual paper method, unchanged - still available |

Full parameters and errors: [Commands in detail](Commands-Detail).

### The automatic contact profile `ke-stock-v1`

Every safety value of an automatic touch comes from a versioned profile in
`nebulaos_raw_contact.py`; a profile id named in a config file is not enough on its own.

| Field | Value | Meaning |
|---|---|---|
| `polarity` | -1 | On the KE, pressing on the bed *lowers* the counts |
| `trigger_counts` | 6400 | Signal change that counts as contact |
| `safety_counts` | 171441 | MCU safety window around the reference (about 2000 g on the development printer) |
| `max_reference_drift_counts` | 6429 | How far the zero before a touch may be from the stored reference (about 75 g) |
| `expected_sps` / `sps_tolerance` | 80 / 25 % | Sample rate the HX711 must deliver |
| `max_idle_sigma` | 640 | Idle noise limit: the trigger must be at least 10x the noise |
| `max_idle_drift_per_s` | 800 | Idle drift limit per second |

The values come from supervised comparisons on the development printer (`counts_per_gram`
85.72, so 6400 counts is about 75 g) and from nine recorded contact traces in
`extras/testdata/`. They are not a multi-unit force qualification: a load cell much more or less
sensitive than this one gets a lower or higher contact force. That is what Manual mode is for.

Before each automatic touch (`_arm_raw()`):

1. a quiet window is taken; if it is still settling (noise or drift above the profile), it is
   measured again in further windows up to a time limit - a real sensor fault refuses at once;
2. the window's median becomes the tare, and must be within `max_reference_drift_counts` of the
   stored reference;
3. the MCU gets the raw safety window `reference ± safety_counts`, which must lie inside the
   sensor's range;
4. the MCU trigger is set to `|counts - tare| >= trigger_counts`. Using the absolute value means
   a signal in the unexpected direction can only stop the nozzle earlier.

There is no fallback: if any of these fails, the touch is refused.

### Where the state lives

| What | Where | Written by |
|---|---|---|
| The Z offset | `[bltouch] z_offset` in the `SAVE_CONFIG` block of `printer.cfg` | `SAVE_CONFIG` after a calibration. The firmware's `machine.cfg` deliberately does not set it |
| The bed mesh | `[bed_mesh default]` in the `SAVE_CONFIG` block | `SAVE_CONFIG` |
| Mode, last automatic check, unloaded reference | `/usr/data/nebulaos/printer_data/calibration/load_cell_mode.json` | `nebulaos_load_cell_mode.py` |
| Bottle calibration | `counts_per_gram`, `reference_tare_counts` in the `SAVE_CONFIG` block | The bottle calibration's save step |
| Z offset adjustment | `/opt/printer_data/config/z_offset_adjust.json` | `nebulaos_z_adjust.py` |
| Auto Calibration progress | `/usr/data/nebulaos/printer_data/calibration/auto_calibrate_journal.json` | `nebulaos_calibration_journal.py` |

The JSON files are written atomically (temporary file, `fsync`, rename, `fsync` of the
directory). A missing or damaged file falls back to the default (Automatic, never checked; or an
adjustment of 0) with a warning - Klipper always starts.

### Configuration

`[nebulaos_calibration]` (Z-offset part), defaults from the module:

| Option | Default | Meaning |
|---|---|---|
| `reference_x`, `reference_y` | none (firmware sets 20, 25) | Where the Z offset is measured |
| `pro_cnt` | 3 | Touches per measurement |
| `horizontal_move_z` | 5 mm | Travel height |
| `established_contact_margin_mm` | 1.0 | Floor below the predicted contact |
| `bootstrap_contact_envelope_mm` | 5.0 | Floor below the CR Touch reading when there is no Z offset yet |
| `down_min_z` | 10 mm | Absolute depth limit below Z 0 (the axis's `position_min` still applies) |
| `max_abs_fit_delta` | 0.5 mm | Per touch: largest accepted fit correction |
| `min_accepted_samples` | 2 | Touches that must pass |
| `max_repeatability_range` | 0.15 mm | Spread of the accepted touches |
| `max_repeatability_stddev` | 0.06 mm | Standard deviation of the accepted touches |
| `max_offset_correction_mm` | 2.0 mm | Largest change from the previous Z offset |
| `z_offset_reference_temp` | 150 °C | Nozzle temperature for the touches |
| `pid_bed_target` | 65 °C | Bed temperature during Auto Calibration |
| `thermal_soak_seconds` | 15 s | Extra wait after reaching temperature |
| `bed_mesh_profile` | `default` | Profile the mesh is saved under |

`[nebulaos_z_offset_probe]` (the firmware's `z_offset_probe.cfg`): `sensor_type: hx711`,
`dout_pin: PC6`, `sclk_pin: PA4`, `contact_speed: 2.0`, `retract_dist: 2.0`,
`retract_speed: 10.0`, `trigger_force: 75` (g, Manual mode), `force_safety_limit: 2000` (g,
Manual mode), `tare_time: 0.067` (s). `contact_detection_mode` (default `auto` = follow the
user's choice) can force `calibrated_force` or `raw_contact` for development; never write it into
the image-owned file.

All options with their allowed ranges: [Configuration](Configuration).

### What the load cell is not used for

The CR Touch remains Klipper's probe for Z homing and the bed mesh. `nebulaos_z_offset_probe` does
not register itself as a Klipper `probe`. The load cell is used for the Z-offset touch and the
nozzle clean, nothing else. Automatic axis-twist calibration with the load cell was a deliberate
product decision against; upstream's manual `AXIS_TWIST_COMPENSATION_CALIBRATE` is unaffected.

### Tests

`test_nebulaos_probe_pair.py`, `test_nebulaos_calibration.py`, `test_nebulaos_raw_contact.py`,
`test_nebulaos_load_cell_mode.py`, `test_nebulaos_load_cell_calibration.py`,
`test_nebulaos_z_adjust.py` and the `test_z_offset_probe_*` files run the logic offline with fakes
from `prtouch_test_support.py`; `test_nebulaos_raw_contact.py` replays the recorded contact traces
in `extras/testdata/`. See [Testing](Testing).

### See also

- [Architecture](Architecture) - how a calibration runs end to end and how its state survives the
  restart.
- [Power-loss recovery](Power-Loss-Recovery) - why a changed Z offset blocks a resume.
