# Testing Plan: Limelight + Auto-Aim + Shooter Lookup

## Code Changes Already Made

| File | Fix |
|---|---|
| `SwerveSub.java` | Fixed inverted alliance logic (Red->wpiRed, Blue->wpiBlue) |
| `SwerveSub.java` | Removed broken manual 180 flip of pose |
| `SwerveSub.java` | Added null/tagCount check before filtering |
| `SwerveSub.java` | Feeds gyro heading to Limelight before each measurement |
| `SwerveSub.java` | Enabled fusion for both LimelightFront and LimelightBackLeft |
| `RunShooterCMD.java` | Moved distance lookup from constructor to `initialize()` |
| `AdjustHoodCMD.java` | Moved distance lookup from constructor to `initialize()` |
| `HoodSub.java` | Fixed `MathUtil.clamp()` result being discarded |

---

## Info Needed From You

### 1. Limelight Mount Offsets

Already configured in the Limelight web UI (`http://limelight-track.local:5801`). If pose looks wrong during Step 2, double-check the offsets match the CAD.

### 2. Shooter Lookup Table Data (from on-robot testing)

This is collected AFTER Limelight pose is validated (Step 2 below). Stand at known distances, tune until shots land, record what works.

**Format needed:**

```
Distance (ft)  |  Shooter RPM or FPS  |  Hood Angle (degrees, 43-63 range)  |  Notes
---------------|----------------------|-------------------------------------|--------
4              |                      |                                     |
6              |                      |                                     |
8              |                      |                                     |
10             |                      |                                     |
12             |                      |                                     |
```

Minimum 4 distances, ideally 6-8 across your realistic shooting range. The interpolation handles in-between values.

**Important:** Record the distance as reported by SmartDashboard (from Limelight/pose), not from a tape measure. The robot will use its own distance estimate in a match.

### 3. Hub/Target Position

The code currently has the hub at `(8.23, 8.23)` meters (blue alliance origin). Verify this matches the actual 2026 field target location. If different, tell me the correct X, Y coordinates.

---

## Testing Steps

### Step 1: Limelight Connectivity (no driving needed)

1. Power on the robot, connect your laptop to the robot network
2. Open `http://limelight-track.local:5801` (front camera)
3. Confirm the dashboard loads and shows a camera feed
4. Set pipeline to AprilTag detection (pipeline 0)
5. **Enter the mount offsets from the table above** in the camera settings
6. Hold an AprilTag in front of the camera, confirm it detects and reports a botpose

If a camera dashboard doesn't load, check:
- Is the camera powered? (green LEDs?)
- Is the camera name in the Limelight UI matching the code? ("limelight-track")
- Is the camera on the same network subnet as your laptop?

### Step 2: Validate Pose Estimation (drive the robot)

1. Deploy the updated code: `JAVA_HOME=~/wpilib/2026/jdk ./gradlew deploy`
2. Open SmartDashboard or Shuffleboard on the Windows laptop
3. Place the robot at a known field position (e.g., a corner or against a wall)
4. Enable the robot, watch SmartDashboard for:
   - `limelight-track_tagCount` — should show 1+ when tags visible
   - `limelight-track_ambiguity` — should be < 0.7 (lower is better)
   - `LL_Dist_ft` — distance to hub in feet (should change as you move)
   - `RobotLocation` — should roughly match where you placed the robot
   - `Field` widget — robot icon should appear near its real position

5. Drive slowly around the field near AprilTags. The pose on the field widget should track smoothly.

**What to look for:**
- Pose jumping wildly = mount offsets are wrong, go back to Step 1
- Pose consistently offset by a fixed amount = mount offsets need adjustment
- Pose only updates when facing certain directions = that camera's offset may be wrong
- No `_tagCount` data appearing = camera not detecting tags (check pipeline, lighting, tag size)

### Step 3: Test Auto-Aim (SwerveLimelightLockCMD)

1. Point the robot so the front Limelight can see an AprilTag near the target
2. Hold button 7 (kPrepareShotButton) — the robot should rotate to center the tag in the camera view
3. Evaluate the response:
   - **Too slow / doesn't reach target**: increase `autoOrientKp` (currently 0.02, try 0.04)
   - **Oscillates around target**: decrease `autoOrientKp` (try 0.01)
   - **Overshoots then corrects**: current value may be fine, just needs time to settle

Tell me the behavior you see and I'll adjust the gain.

### Step 4: Collect Shooter Data

**Prerequisites:** Limelight pose is working (Steps 1-2 pass). You need game pieces to shoot.

**How this works:** You are NOT trying to hit distances from the existing lookup table. The existing table is all placeholders. You are BUILDING a new table from scratch. Place the robot wherever makes sense, read the distance the robot reports, tune until shots land, and write it down.

1. Open SmartDashboard, find `LL_Dist_ft` — this is your live distance readout
2. Place the robot at a comfortable close range where you can easily make shots
3. Watch these dashboard values:
   - `LL_Dist_ft` — the distance to record (this is what the robot sees, use THIS number)
   - `DesiredVelocity` — what the shooter is targeting
   - `DesiredHoodAngle` — what the hood is targeting
   - `isAtSetAngle` — hood has arrived at position
4. Use the existing close-shot button (button 1) to start with the current hardcoded values (45 FPS, 56 deg)
5. If the shot misses, manually adjust via the tunable hood angle / shooter velocity dashboard widgets until shots land consistently
6. Once you're making 3/5+ shots, write down:
   - **`LL_Dist_ft` value from the dashboard** (NOT a tape measure — the robot uses this estimate in a match)
   - The shooter velocity (FPS) that worked
   - The hood angle (degrees) that worked
7. Move the robot to a different distance, repeat from step 4
8. Continue until you've covered your full shooting range (close to far)

**You do not need to be at evenly spaced or round-number distances.** If you end up at 4.3 ft, 7.1 ft, 9.6 ft — that's fine. The interpolation handles everything in between.

Minimum 4 distances, ideally 6-8 for good coverage.

**Tips:**
- Use a fully charged battery for the whole session, or note voltage if you swap
- Wait for the shooter to reach target speed before feeding (watch the velocity telemetry)
- If hood angle exceeds 43-63 degree range, you've hit the mechanical limits — don't go further
- Start close and work your way back — it's easier to find working values near the target first

### Step 5: Give Me The Data

Once you have a table like this:

```
LL_Dist_ft  |  Shooter FPS  |  Hood Angle (deg)
--------------|---------------|------------------
4.3           |  42           |  58
6.0           |  55           |  53
8.2           |  68           |  48
10.5          |  80           |  45
```

Send it to me and I'll replace the placeholder lookup table in `ShooterLookup.java` with your real values.

---

## Quick Reference: SmartDashboard Keys to Watch

| Key | What It Shows |
|---|---|
| `limelight-track_tagCount` | Tags seen by front camera |
| `limelight-track_ambiguity` | Front camera pose confidence (lower = better) |
| `RobotHeading` | Current gyro heading (degrees) |
| `RobotLocation` | Current estimated X,Y position |
| `Field` | Visual field widget showing robot pose |
| `LL_Dist_ft` | Limelight distance to target in feet (use for shooter calibration) |
| `LL_Dist_in` | Limelight distance to target in inches |
| `DesiredVelocity` | Shooter target velocity (FPS) |
| `DesiredHoodAngle` | Hood target angle (degrees) |
| `TunableHoodAngle` | Manual hood angle override |
| `CurrentLeftPulseWidth` | Left hood servo actual position |
| `CurrentRightPulseWidth` | Right hood servo actual position |
| `isAtSetAngle` | Whether hood has reached target |
