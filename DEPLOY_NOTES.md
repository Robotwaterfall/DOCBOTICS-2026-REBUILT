# DocBotics Deploy Notes

## Build

```bash
cd /Users/alex/Documents/DOCBOTICS-2026
JAVA_HOME=~/wpilib/2026/jdk ./gradlew build
```

## Deploy to Robot

Connect to the robot's network first (USB or WiFi), then:

```bash
JAVA_HOME=~/wpilib/2026/jdk ./gradlew deploy
```

### Network options
- **USB**: Plug USB-B into roboRIO. Robot is at `172.22.11.2`
- **WiFi**: Connect to robot radio. Robot is at `10.TE.AM.2` (replace TE.AM with team number, e.g. `10.12.34.2` for team 1234)

## Optional: Set JAVA_HOME permanently

Add this to `~/.zshrc` so you don't need to type it every time:

```bash
export JAVA_HOME=~/wpilib/2026/jdk
```

Then just `./gradlew deploy` works on its own.

## Enable the Robot

Deploy from Mac, then use the **Windows laptop** with FRC Driver Station to enable. Driver Station is Windows-only.

## Limelight Web UI

Open in any browser while connected to the robot network:

```
http://limelight.local:5801
```

Use this to check camera pipelines, AprilTag detection, and mount offsets (height, angle, forward/side position in meters).

## AdvantageScope

Launch from `~/wpilib/advantagescope`.

1. Connect your laptop to the robot network (USB or WiFi)
2. In AdvantageScope, click **File > Connect to Robot** (or the plug icon)
3. It auto-discovers the roboRIO — if not, enter the robot IP (`172.22.11.2` for USB, `10.TE.AM.2` for WiFi)
4. Once connected, all NetworkTables values stream in live

### Recording & replay
- AdvantageScope records automatically while connected
- Use **File > Save Log** to save a `.wpilog` file for later review
- Open saved logs with **File > Open Log** on any computer (doesn't need robot connection)

### Plotting shooter data
- Drag `shooterLeadVelocityFPS`, `shooterFollowerRightVelocityFPS`, `shooterFollowerLeftVelocityFPS`, and `currentDesiredVelocityFPS` onto the same line graph
- Fire a few shots, then scrub the timeline to see the velocity dip on ball contact
- Zoom into the dip to see per-motor recovery time

## Dashboards (pick one for live telemetry)

All installed with WPILib, all read from the same NetworkTables data:

| App | Where | Notes |
|---|---|---|
| Elastic | `~/wpilib/elastic` | Modern UI, drag-and-drop |
| Shuffleboard | WPILib tools menu | Drag-and-drop widgets, tabs |
| SmartDashboard | WPILib tools menu | Simple key-value display |
| Glass | WPILib tools menu | Good for field visualization |

Connect the same way as AdvantageScope — just be on the robot network.

## Build Warnings

The project compiles with 7 deprecation warnings (REV `ResetMode`/`PersistMode` and CTRE `Pigeon2` constructor changes). These are cosmetic and don't affect functionality.

## Installed Software

| Tool | Location |
|---|---|
| WPILib 2026 (tools only) | `~/wpilib/2026/` |
| Bundled JDK 17 | `~/wpilib/2026/jdk/` |
| Project repo | `/Users/alex/Documents/DOCBOTICS-2026/` |
| Branch | `GeoriganCollege-Stable` |
