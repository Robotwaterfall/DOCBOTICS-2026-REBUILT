# DOCBOTICS 2026 - REBUILT

## Project Overview

FRC (FIRST Robotics Competition) robot code for team DocBotics. Java, WPILib command-based architecture, built with Gradle.

**Active branch:** `GeoriganCollege-Stable` (competition-prep branch for Georgian College event)

## 2026 Game: REBUILT

Two alliances of 3 robots score **Fuel** (foam balls, 5.91" diameter) into their **Hub**, navigate field obstacles (Bumps and Trenches), and **climb the Tower** in endgame.

- **Auto:** 15 seconds, autonomous
- **Teleop:** 2 minutes 15 seconds, driver-controlled
- **Endgame:** Final 30 seconds, climbing

### Scoring
- **Fuel in active Hub:** 1 point each (auto and teleop)
- **Tower climbing:** 10/20/30 points for Low/Mid/High rung
- **Ranking points:** Energized (100+ fuel), Supercharged (360+ fuel), Traversal (50+ tower points)

### Hub Shift System
Hubs alternate between active/inactive during teleop. The alliance with fewer auto points gets first activation. Only fuel in the active hub counts.

### Field Dimensions
- 16.541m x 8.069m (651.2" x 317.7")
- Blue hub center: (4.63, 4.03) meters
- Red hub center: (11.92, 4.03) meters
- `PoseManager.getAllianceHubPose2d()` handles red/blue flipping via `FlippingUtil`

### AprilTag Layout (32 tags)
Tags are in the WPILib field layout `2026-rebuilt-welded.json`. Key groups:
- **Blue hub ring (tags 18-21, 24-27):** Around (4.0-5.2, 3.4-4.6)m, z=1.124m
- **Red hub ring (tags 2-5, 8-11):** Around (11.3-12.5, 3.4-4.6)m, z=1.124m
- **Blue tower/wall (tags 17, 22, 23, 28):** x~4.6m, z=0.889m
- **Red tower/wall (tags 1, 6, 7, 12):** x~11.9m, z=0.889m
- **Blue alliance wall (tags 29-32):** x~0.008m, z=0.552m
- **Red alliance wall (tags 13-16):** x~16.533m, z=0.552m

## Robot Hardware

Based on the WCP 2026 Competitive Concept, but with different motors and hardware.

| Mechanism | Motors | Controller |
|---|---|---|
| Swerve drive (4 modules) | 4x SparkMax (drive) + 4x SparkMax (turn) | NEO brushless |
| Swerve encoders | 4x CANcoder (CAN 29, 26, 27, 28) | |
| Gyro | Pigeon2 (CAN 21) | |
| Intake pivot | 1x SparkMax (CAN 2) | Software PID |
| Intake rollers | 1x TalonFX (CAN 3) | Velocity control |
| Conveyor | 1x SparkMax (CAN 7) | Percent output |
| Indexer/feeder | 1x TalonFX (CAN 25) | Percent output |
| Shooter (3 wheels) | 3x TalonFX (CAN 24, 23, 22) | Velocity control |
| Hood | REV ServoHub (CAN 30), 2 channels | Linear actuators, cubic angle-to-extension mapping |
| Limelight (front) | "limelight-track" | AprilTag pipeline 0 |
| Limelight (back-left) | "limelight-left" | Currently disabled in code, enable when ready |

All CAN devices on the roboRIO bus ("rio"), no CANivore.

## Build & Deploy

```bash
# Build
JAVA_HOME=~/wpilib/2026/jdk ./gradlew build

# Deploy to robot (connect to robot network first)
JAVA_HOME=~/wpilib/2026/jdk ./gradlew deploy
```

Or set `export JAVA_HOME=~/wpilib/2026/jdk` in `~/.zshrc` and just use `./gradlew deploy`.

Driver Station (Windows only) required to enable the robot. Deploy can be done from any OS.

## Architecture

Standard WPILib command-based pattern:
- `RobotContainer.java` — creates subsystems, binds controller buttons to commands
- `Constants.java` — all hardware IDs, PID gains, dimensions, button mappings
- `subsystems/` — one class per mechanism, owns motors/sensors
- `commands/` — actions that use subsystems; `commandgroups/` for multi-step sequences
- `util/PoseManager.java` — distance/heading calculations from pose estimator to hub
- `util/ShooterLookup.java` — interpolation table: distance -> (shooter FPS, hood angle)
- `config/LimelightHelpers.java` — vendored Limelight API v1.13, do not edit

## Key Systems

### Limelight Pose Fusion (SwerveSub.java)
- Feeds gyro heading to Limelight via `SetRobotOrientation` (enables MegaTag2)
- Calls `getBotPoseEstimate_wpiRed/Blue` based on alliance
- Filters by tag count >= 1, ambiguity < 0.7, latency > 0.01
- Front camera std devs: (0.5, 0.5, 1000) — trust XY, ignore rotation
- Back cameras use (1.0, 1.0, 1000) — slightly less trust

### Auto-Aim (SwerveLimelightLockCMD.java)
- Uses raw Limelight TX for rotation, driver controls translation
- Proportional gain: `autoOrientKp = 0.02`
- Allows full field-relative driving while robot stays pointed at target

### Shooter Lookup (ShooterLookup.java)
- TreeMap interpolation: distance (feet) -> (hood angle degrees, shooter FPS)
- **Current values are placeholders** — need real calibration data
- Hood range: 43-63 degrees (HoodConstants)
- RunShooterCMD and AdjustHoodCMD compute distance in initialize(), not constructor

### Controller Layout (PS5, port 0)
- Left stick: drive translation (X/Y)
- Right stick (axis 2): rotation
- Button 1: close shot (hardcoded values)
- Button 2: far shot (hardcoded values)
- Button 5: outtake
- Button 6: intake
- Button 7: Limelight lock (auto-aim + drive)
- Button 8: fire shot sequence
- Button 10: gyro reset
- Button 11: slow mode

## Known Issues / TODOs

- Shooter lookup table needs real calibration data (all entries use same velocity)
- Back-left Limelight ("limelight-left") disabled — enable after front camera validated
- Third Limelight ("limelightbackright") referenced but may not be installed
- Deadband is 0.5 (very high — typical is 0.05-0.15)
- Button 8 has duplicate binding in RobotContainer
- IntakePitcherSub runs PID in periodic() even when no command is active
- Several TODO comments remain in Constants.java for PID tuning
- `kWheelDiameterInches` in SwerveModuleConstants is actually in meters (misleading name)
