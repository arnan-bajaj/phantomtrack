# Anonymous tracking: dropping the "we know who every sighting is" assumption

Branch `anon-tracking`. All numbers are from the simulator: 180 s runs, 5 seeds per camera row,
two alliance partners on the field. Reproduce them with the commands at the end.

## The short version

Until now every camera sighting came with the right opponent id (`Measurement.id`). A real
camera gives anonymous boxes: opponents **and** our own partners, a bumper color that is
sometimes wrong, false positives, and robots hidden behind other robots. This branch simulates
that camera and adds a multi-target tracker that works without ids. Results:

1. **With the Realistic camera, positional defense barely notices.** Line mirror + hold (auto =
   threat) slows the opponents about as much as with free ids (anonymous vs known-ID):
   - open field: +15% vs +15% (Cyclers), +9% vs +10% (Evaders);
   - choke: +21% vs +27% (Cyclers), +16% vs +14% (Evaders).

   It does not care *which* robot is which, only where the threat is.
2. **Man-to-man costs the driver attention.** Cut-off + hold locked on one opponent:
   - open field, holds up: +24% vs +26% (Cyclers), +19% vs +16% (Evaders);
   - choke, loses 12–16 points: +26% vs +42% (Cyclers), +21% vs +33% (Evaders).

   The driver has to point the assist back at the right robot about 9 times a minute (every
   ~7 s), because the guarded robot's track keeps breaking.
3. **The guarded robot's identity is lost whenever it is out of view for more than about a
   second.** It goes back to its pickup about 9 m away on every cycle, past the camera's range.
   Re-acquisition halves the damage (19.5 → 9.1 re-points per minute). A 360° camera halves it
   again (4.5 per minute). **Camera coverage, already the first thing to buy, matters even more
   now.**
4. **With the Bad camera, anonymous tracking falls apart.** Opponent coverage halves (36% → 18%)
   and most of the remaining defense effect goes with it. At the choke against Evaders it drops
   to nothing (−2%, 0%).
5. **Partners are rarely mistaken for opponents.** A fresh opponent track sits on one of our
   partners 1.6 s per minute (Realistic), about 3% of the time. My guess, not measured: most of
   it is a track sliding from an opponent onto a partner that passes close by, until a few
   partner-colored labels flip it.
6. **Occlusion alone is cheap.** Hiding robots behind robots costs about 6 points of coverage
   (66% → 60%) and no measurable defense effect.
7. **Pins don't get worse with anonymous tracking** (1.9 ± 0.3 vs 1.7 ± 0.3 per run pooled,
   Realistic). At the choke every camera tracker lets about 1–6 pins per run through (perfect
   tracking: 0–1). That is the open contact-sensing problem the README already describes, not
   something anonymity adds.

## What was built

**Simulator** (`phantom.sim`)

- **Alliance partners** (`Scenarios.three(..., withPartners)`, off by default). Two partners
  mirror the opponents' layout through the center line: they load at the far end (15.04, 1.2)
  and (15.04, 6.9) and score at (3.54, 4.035), so they cross our defensive area and the
  opponents' lanes. They always drive like Evaders (steer around robots, 0.25 s human reaction),
  because a teammate does not drive through its own defender. They collide, block and get seen,
  but never count as opponents (not in pins, not in opponent contact time).
- **`AnonymousSensor`**. Same camera model as `NoisySensor` (field of view, range, Gaussian noise,
  latency, dropped detections, frame rate), plus:
  - an alliance label (opponent / partner) that is flipped with a configurable error rate;
  - clutter: a Poisson number of false detections per frame, uniform over the area of the
    field-of-view sector (points outside the field are dropped, as a pipeline would);
  - occlusion: a robot is not seen if the line from our camera to its center passes within one
    robot radius (0.45 m) of a closer robot;
  - our own robot: the detector sometimes fires on our own bumpers, and the pipeline drops every
    detection inside our bumpers (closer than 0.45 m to our center). Noisy ones can leak out;
  - detections are shuffled and every frame is delivered, even an empty one.

| Preset | Camera (unchanged) | Wrong color | False detections | Own bumpers detected | Occlusion |
| --- | --- | --- | --- | --- | --- |
| Realistic | 140°, 0.10 m, 80 ms, 15% dropped, 8 m, 30 Hz | 5% | 0.1 per frame (3 per s) | 10% of frames | on |
| Bad | 70°, 0.25 m, 150 ms, 35% dropped, 6 m, 15 Hz | 15% | 0.5 per frame (7.5 per s) | 30% of frames | on |

The added numbers are **guesses**, like the camera numbers, until the real pipeline is measured.

**Tracker** (`phantom.core.tracking`, zero dependencies, `OpponentTracker` unchanged)

- `Hungarian`: optimal assignment, O(n^3), rectangular matrices, own implementation.
- `Gating`: Mahalanobis distance, 2 degrees of freedom, 99% gate = 9.210 (chi-square table;
  closed form −2 ln 0.01 = 9.2103).
- `ColorBelief`: alliance color as log-odds, ±ln((1 − e)/e) per label, clamped at ±10 so a
  settled belief can still flip (4 wrong labels in a row at e = 5%, 6 at 15%).
- `MOfN`: confirmation, default 3 of 5.
- `MultiTargetTracker`: every frame, gate → cost = d² + ln det S − 2 ln P(label | track) →
  global nearest neighbor (Hungarian) → Kalman update (`AxisKalman`, same filter and numbers as
  `KalmanOpponentTracker`). Life cycle: tentative → confirmed → coasting → deleted after 1 s.
  Only tracks believed to be opponents (P > 0.5) are reported. Confidence = P(opponent) ×
  0.25 / (0.25 + position variance). Frames are applied at capture time; old frames are
  dropped. For one robot it produces exactly the same numbers as `KalmanOpponentTracker` (tested).
- **Re-acquisition** (the man-to-man fix), in the tracker so every user gets it:
  - *by proximity*: a new reported track takes over the id of a lost track (or of a reported
    track that is still coasting, which is then deleted as a stale copy) if it is within 2.5 m
    of where that track would be after at most 0.5 s more of its last velocity, within 2 s;
  - *by elimination*: if a new opponent track got no id that way, and there are at most 3
    opponent tracks with it, and exactly one id was lost in the last 4 s, it takes that id. FRC
    alliances are 3 robots, so there is nobody else it can be.

**Focus without ids.** With known ids, "locked on #0" is just id 0. Anonymous track ids mean
nothing in advance, so the sim plays a driver who points the assist at the track on opponent #0
and points it again 0.5 s after the assist loses it or ends up on the wrong robot. The number of
re-points per minute is how much attention man-to-man costs the driver. This simulated driver
always knows which robot is #0 (a human can read bumper numbers), so it is optimistic.

## Results

### Defense effect (5 seeds, ± = standard error over seeds)

"Known-ID" is the old model: the old camera, ids handed over. "Known-ID + occlusion" is the new
camera's occlusion with ids still handed over, which separates the two effects. "Perfect" is
one deterministic run with ground truth.

#### Line mirror + hold, auto = threat (slowed = all three opponents' cycles)

| Opponents | Field | Perfect | Camera | Known-ID | Known-ID + occlusion | **Anonymous** | Pins/run K / K+o / A |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Cyclers | Open | +15% | Realistic | +15% ± 1 | +14% ± 0 | **+15% ± 1** | 0.8 / 1.0 / 0.6 |
| Cyclers | Open | +15% | Bad | +10% ± 1 | +11% ± 1 | **+10% ± 1** | 0.6 / 1.2 / 0.4 |
| Cyclers | Choke | +24% | Realistic | +27% ± 3 | +24% ± 2 | **+21% ± 3** | 2.8 / 3.2 / 3.4 |
| Cyclers | Choke | +24% | Bad | +22% ± 3 | +22% ± 3 | **+12% ± 2** | 6.4 / 4.8 / 1.6 |
| Evaders | Open | +17% | Realistic | +10% ± 1 | +10% ± 1 | **+9% ± 1** | 0.2 / 0.0 / 0.0 |
| Evaders | Open | +17% | Bad | +10% ± 1 | +8% ± 0 | **+9% ± 1** | 0.0 / 0.0 / 0.0 |
| Evaders | Choke | +18% | Realistic | +14% ± 3 | +26% ± 13 | **+16% ± 5** | 2.6 / 4.2 / 3.6 |
| Evaders | Choke | +18% | Bad | +8% ± 7 | +4% ± 4 | **−2% ± 2** | 2.4 / 2.4 / 0.8 |

#### Cut-off + hold, locked on #0 (slowed = guarded opponent #0)

| Opponents | Field | Perfect | Camera | Known-ID | Known-ID + occlusion | **Anonymous** | Pins/run K / K+o / A |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Cyclers | Open | +30% | Realistic | +26% ± 2 | +25% ± 1 | **+24% ± 2** | 1.6 / 1.0 / 2.4 |
| Cyclers | Open | +30% | Bad | +15% ± 1 | +12% ± 1 | **+7% ± 1** | 1.8 / 2.4 / 2.0 |
| Cyclers | Choke | +30% | Realistic | +42% ± 1 | +34% ± 1 | **+26% ± 3** | 3.8 / 4.4 / 4.0 |
| Cyclers | Choke | +30% | Bad | +21% ± 2 | +21% ± 4 | **+29% ± 5** | 6.2 / 5.6 / 6.2 |
| Evaders | Open | +18% | Realistic | +16% ± 1 | +18% ± 1 | **+19% ± 1** | 0.0 / 0.2 / 0.0 |
| Evaders | Open | +18% | Bad | +8% ± 1 | +10% ± 1 | **+7% ± 2** | 0.0 / 0.2 / 0.0 |
| Evaders | Choke | +13% | Realistic | +33% ± 6 | +44% ± 16 | **+21% ± 3** | 2.0 / 1.8 / 1.2 |
| Evaders | Choke | +13% | Bad | +12% ± 4 | +20% ± 4 | **−0% ± 3** | 0.8 / 1.8 / 0.8 |

Read with care: several cells have a large standard error (one jammed run drags a 5-seed mean).
Treat differences under about 2 standard errors as ties. Some camera rows beat "Perfect": the
perfect row is a single run of a chaotic system, and noisy tracking sometimes puts our robot in
a lucky spot (the README already notes this for plain Shadow).

### Tracking quality (pooled: both setups, both opponent kinds, open + choke = 40 runs per row)

| Camera | Tracking | Position RMSE | ID switches / min | False tracks, s/min (fresh) | Opponent coverage | Partner as opponent, s/min (fresh) | Driver re-points / min (Cut-off locked) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Realistic | Known-ID | 0.41 m | 3.4 | 6.6 (0.0) | 66% | 0.7 (0.1) | 0 (ids are given) |
| Realistic | Known-ID + occlusion | 0.45 m | 3.9 | 7.3 (0.0) | 60% | 1.0 (0.1) | 0 (ids are given) |
| Realistic | **Anonymous** | 0.50 m | 33.0 | 13.4 (1.6) | 55% | 2.9 (1.6) | 9.1 |
| Bad | Known-ID | 0.74 m | 10.3 | 9.0 (0.2) | 36% | 2.3 (0.6) | 0 (ids are given) |
| Bad | Known-ID + occlusion | 0.76 m | 10.8 | 8.9 (0.3) | 32% | 2.4 (0.6) | 0 (ids are given) |
| Bad | **Anonymous** | 0.82 m | 30.2 | 25.9 (5.0) | 18% | 5.9 (2.9) | 5.5 |

- **Known-ID "ID switches" are not identity errors.** They come from coasting tracks: for up to
  1 s after its last sighting a track keeps moving at its last velocity, drifts off its robot
  and gets scored onto a different one. Anonymous tracking has ~30 real ones per minute,
  about one per opponent every 6 s.
- **Most false-track time is stale coasting, for both trackers.** Fresh false tracks (phantoms
  and duplicates) are 1.6 s/min with Realistic and 5 s/min with Bad. Uniform, independent clutter
  rarely lands in the same spot 3 frames out of 5, so it seldom becomes a confirmed track on its
  own (an estimate, not measured separately). Real clutter is not like that (see the list at the
  end).
- Bad's re-point rate is *lower* only because the target is tracked so rarely that there is
  often nothing to point at.

### Re-acquisition on vs off (anonymous, pooled over opponent kinds and fields: 20 runs per cell)

| Setup | Camera | Slowed on / off | ID switches/min on / off | Driver re-points/min on / off | Coverage on / off | Pins on / off |
| --- | --- | --- | --- | --- | --- | --- |
| Line mirror + hold, auto=threat | Realistic | +15% ± 2 / +13% ± 2 | 32.7 / 63.1 | – | 55% / 54% | 1.9 / 2.2 |
| Line mirror + hold, auto=threat | Bad | +7% ± 1 / +6% ± 2 | 27.2 / 40.4 | – | 16% / 17% | 0.7 / 1.0 |
| Cut-off + hold, locked #0 | Realistic | +23% ± 1 / +22% ± 2 | 33.4 / 72.5 | 9.1 / 19.5 | 55% / 58% | 1.9 / 2.0 |
| Cut-off + hold, locked #0 | Bad | +11% ± 3 / +37% ± 28 | 33.3 / 51.7 | 5.5 / 7.1 | 19% / 22% | 2.2 / 1.6 |

Re-acquisition halves ID switches and the driver's re-points. It does **not** measurably change
how much the opponents are slowed: the simulated driver fixes a lost lock within 0.5 s either
way. (The +37% ± 28 cell is one jammed run.) What it buys is driver attention: a re-point every
7 s instead of every 3 s.

**Why the guarded track still breaks ~9 times a minute.** A one-off diagnostic, a throwaway
script and not in the repo, followed opponent #0 with the Realistic camera. About 5 of its
~10 id changes per minute happen after it was out of view for over a second: it goes back to
its pickup ~9 m away, past the 8 m range, and comes back somewhere else. About 3 per minute are
swaps with another opponent when they cross. About 0.2 per minute are swaps with a partner.

### How the tracker defaults were chosen (`--tracker-sweep 180 5`)

Each row pools both opponent kinds, open + choke, and both cameras (80 runs).

| Re-acquisition | Line mirror auto: ID sw/min | Cut-off locked: ID sw/min | Cut-off locked: re-points/min | Cut-off locked: slowed |
| --- | --- | --- | --- | --- |
| off | 51.8 | 62.1 | 13.3 | +29% ± 14 (one jam) |
| 1.5 m, 0.5 s lead, velocity gate 3 m/s (first guess) | 41.2 | 47.4 | 10.5 | +19% ± 3 |
| 1.5 m, no velocity gate | 36.6 | 42.0 | 9.1 | +21% ± 4 |
| 2.5 m, no velocity gate | 33.9 | 36.2 | 7.9 | +16% ± 2 |
| 3.5 m, 1.0 s lead, 3 s window | 32.2 | 35.6 | 8.2 | +17% ± 2 |
| **2.5 m + elimination 4 s (default)** | **30.0** | **33.3** | **7.3** | +17% ± 2 |
| 2.5 m + elimination 8 s | 32.6 | 35.6 | 7.6 | +18% ± 2 |

- The brief asked for handoff "by proximity + velocity". The velocity-*difference* gate made
  things worse: a track confirmed 3 frames ago has a velocity estimate too noisy to compare, so
  the gate rejected good handoffs. Velocity is still used, to predict where the lost robot should
  be. The gate stays available (`handoffSpeedDiff`) but is off by default.
- 3.5 m was about as good as 2.5 m. I kept the smaller radius because a wrong handoff gives the
  guarded robot's id to a different robot. A longer lead (1.0 s instead of 0.5 s) did not help
  (36.9 and 43.2 ID switches/min at 1.5 m).

Confirmation (M of N), anonymous tracking, both setups:

| Camera | M of N | Slowed (all, Line mirror auto) | ID sw/min | Coverage | Fresh false s/min | Re-points/min |
| --- | --- | --- | --- | --- | --- | --- |
| Realistic | **3 of 5** (default) | +15% ± 2 | 32.7 | 55% | 1.9 | 9.1 |
| Realistic | 2 of 4 | +14% ± 2 | 31.9 | 58% | 1.9 | 8.0 |
| Realistic | 2 of 3 | +15% ± 2 | 30.7 | 58% | 1.9 | 8.2 |
| Bad | **3 of 5** (default) | +7% ± 2 | 27.2 | 16% | 4.8 | 5.5 |
| Bad | 2 of 4 | +7% ± 2 | 36.6 | 18% | 7.3 | 5.7 |
| Bad | 2 of 3 | +8% ± 2 | 33.5 | 17% | 7.1 | 6.1 |

3 of 5 stays. Looser confirmation helps a little at 30 Hz, but with the Bad camera's clutter it
makes more junk tracks and more ID switches. Confirmation is not the bottleneck.

Camera coverage (Realistic camera, 140° vs 360° of view; known-ID here is the old camera model):

| Setup | View | Tracking | Slowed | ID sw/min | Coverage | Re-points/min |
| --- | --- | --- | --- | --- | --- | --- |
| Line mirror + hold, auto=threat | 140° | Known-ID | +16% ± 2 | 2.2 | 68% | – |
| Line mirror + hold, auto=threat | 140° | Anonymous | +15% ± 2 | 32.7 | 55% | – |
| Line mirror + hold, auto=threat | 360° | Known-ID | +29% ± 10 | 0.0 | 94% | – |
| Line mirror + hold, auto=threat | 360° | Anonymous | +18% ± 2 | 13.4 | 86% | – |
| Cut-off + hold, locked #0 | 140° | Known-ID | +29% ± 3 | 4.6 | 63% | – |
| Cut-off + hold, locked #0 | 140° | Anonymous | +23% ± 1 | 33.4 | 55% | 9.1 |
| Cut-off + hold, locked #0 | 360° | Known-ID | +32% ± 3 | 0.1 | 93% | – |
| Cut-off + hold, locked #0 | 360° | Anonymous | +24% ± 1 | 16.2 | 82% | 4.5 |

Where coverage goes, one change at a time (Line mirror + hold, auto = threat):

| Camera | Pipeline | Coverage | RMSE | Slowed (all) |
| --- | --- | --- | --- | --- |
| Realistic | known ids, old camera (no occlusion) | 68% | 0.40 | +16% ± 2 |
| Realistic | known ids + occlusion | 62% | 0.44 | +18% ± 3 |
| Realistic | anonymous (the full new camera) | 55% | 0.50 | +15% ± 2 |
| Realistic | anonymous, no false/self detections, no color errors | 58% | 0.48 | +16% ± 3 |
| Realistic | anonymous, same, confirmed on the first detection | 60% | 0.48 | +15% ± 2 |
| Bad | known ids, old camera (no occlusion) | 37% | 0.75 | +13% ± 2 |
| Bad | known ids + occlusion | 33% | 0.76 | +11% ± 2 |
| Bad | anonymous (the full new camera) | 16% | 0.83 | +7% ± 2 |
| Bad | anonymous, no false/self detections, no color errors | 22% | 0.77 | +7% ± 2 |
| Bad | anonymous, same, confirmed on the first detection | 27% | 0.79 | +10% ± 2 |

With the Bad camera the loss splits roughly into thirds:

- **detector errors** (false detections, own-bumper detections, color errors);
- **confirmation latency**: 3 hits at 15 Hz with 35% dropped frames, while robots flicker in
  and out of a 70° view;
- **the rest**: my guess is a feedback effect. When the auto focus switches more, our robot turns
  more, and the 70° camera looks somewhere else. I have not proven this.

### Partners on the field (all trackers)

Our robot touches its own partners 3 s per 180 s run in open field and **20 s per run at the
choke**, with perfect tracking too (20.6 s). Partners have to use the same single gap our
defender is guarding. Any real game with a central chokepoint will have this, and it is not a
tracking problem. Pins at the choke with partners on (about 1–6 per run with a camera) are in
the same range as the README's 1–5 without partners.

## What still differs from a real camera

1. **False positives are friendlier than real ones.** Ours are independent and uniform. Real ones
   repeat in the same place (field elements, reflections, people at the field edge, game pieces),
   which is exactly what beats M-of-N confirmation. A persistent false positive would become a
   confirmed phantom robot.
2. **Color errors are independent per frame here.** Real ones are correlated: a robot with
   unusual bumpers, glare, or a dark corner is misread frame after frame. The log-odds belief
   assumes independence and would be confidently wrong.
3. **Detection probability is a constant** inside a hard field-of-view and range cutoff. Real
   detection rates fall off with distance, angle, motion blur, lighting and partial occlusion.
4. **Occlusion is all-or-nothing**, decided by a ray to the robot's center, with robots as
   circles. Real partial occlusion gives shifted, shrunken boxes (biased positions), and field
   elements block the view too. The choke's walls are see-through here.
5. **Noise is round, Gaussian, independent and unbiased.** Real camera position error grows with
   distance, mostly along the line of sight, and has biases (bumper-height assumptions, robot
   tilt when accelerating).
6. **One combined camera at the robot's center, one detection per robot per frame.** Real robots
   have several offset cameras with blind spots between them, and the same robot seen by two
   cameras gives two detections in one frame. That needs cross-camera fusion before this tracker,
   which does not exist yet.
7. **Constant latency and perfect timestamps.** Real pipelines have jitter, dropped frames and
   coprocessor/roboRIO clock offsets.
8. **Our own pose is exact.** Real field-relative detections inherit our odometry/AprilTag error,
   which shifts every detection together and can move a partner across a gate.
9. **Robots are identical circles with no numbers.** Real robots differ in size and shape, and
   their bumper numbers are readable at close range. Reading them (or any appearance feature)
   would fix most ID switches. This sim can't tell anything apart except color.
10. **Our own bumpers** show up in a fixed fraction of frames at our center. Real self-detections
    depend on where the cameras are mounted, and are usually masked out in the image.
11. **The world is made up:** the partner layout (mirrored, crossing our zone), Evader-style
    partners, scripted opponents, 5 seeds with large spread in some cells, and an optimistic
    simulated driver who always knows which robot is #0 and reacts in exactly 0.5 s.
12. **The tracker was tuned on the same simulation it is judged on.** The margins between
    settings are small, so expect retuning on real data.

## How to reproduce

```bash
./gradlew run --args="--headless"              # includes the anonymous tables (last 3 sections)
./gradlew run --args="--anonymous 180 5"       # only those tables
./gradlew run --args="--tracker-sweep 180 5"   # re-acquisition, confirmation, 360 deg, breakdown
./gradlew run --args="--batch 180 5"           # results/batch.csv, one row per run, all metrics
```

In the sim window: **T** cycles tracking (known-ID → known-ID + occlusion → anonymous), **B**
toggles partners, **F** with anonymous tracking locks the simulated driver on #0, #1, #2 or auto.

**Metric definitions** (`SimSession.Metrics`, the same scorer for every tracker, no ids used):
every 20 ms, reported tracks (extrapolated to now) are matched to real robots, opponents and
partners. Last step's pairs are kept while still within 1.5 m (the CLEAR-MOT rule). Then a
Hungarian on distance with a 1.5 m limit matches the rest.

- **Position RMSE**: over opponent matches.
- **ID switches**: an opponent's matched track id changes from the last one it had.
- **False-track time**: track-seconds with no robot within reach.
- **Partner-as-opponent time**: track-seconds matched to a partner.
- **"Fresh"**: the track's last sighting is at most 0.3 s old.
- **Coverage**: the fraction of time an opponent has a reported track on it.
- **Re-points**: times per minute the simulated driver re-points a locked assist (not the first
  designation).
