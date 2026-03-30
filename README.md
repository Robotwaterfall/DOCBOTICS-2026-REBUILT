
  
  # 🤖 Docbotics 6110 Code Pipeline

## Branch: `limelight-shooter-tuning`

Changes from `GeoriganCollege-Stable` focused on getting Limelight pose estimation and distance-based shooter control working.

### What changed

**Limelight pose fusion (SwerveSub.java)**
- Fixed alliance logic — always use `wpiBlue` since the pose estimator works in blue-origin coordinates
- Feed gyro heading to Limelight before each measurement (enables MegaTag2 for more stable single-tag pose)
- Added null safety and tag count checks
- Enabled front camera fusion in periodic loop
- Added `LL_Dist_ft` and `LL_Dist_in` to dashboard — direct Limelight distance to target for shooter calibration

**Gyro / alliance handling (SwerveSub.java)**
- Gyro reset now sets 180 on red alliance so field-relative driving works correctly on both sides

**Shooter pipeline (RunShooterCMD, ShooterLookup, AdjustHoodCMD)**
- Removed hub XY coordinate dependency — shooter distance now comes directly from Limelight camera-to-target measurement
- Simplified ShooterLookup to distance (ft) -> velocity (FPS) only, removed hood angle from lookup
- AdjustHoodCMD simplified to fixed angle (hood angle variation not needed for shot range)
- RunShooterCMD computes distance in `initialize()` not constructor (was stale when pre-constructed)

**Shooter tuning mode (ShooterTuningCMD.java — new)**
- Hold button 3 to enter tuning mode
- D-pad LEFT/RIGHT adjusts shooter velocity by 5 FPS per press
- Starts at 0 FPS, release button 3 to stop
- Dashboard shows `Tuning_VelocityFPS` for recording calibration data

**Dashboard telemetry (ShooterSub.java)**
- Added left follower motor velocity (was missing)
- Added average velocity across all 3 motors
- Added velocity error (target minus actual) for diagnosing ball compression dips

**Bug fixes**
- HoodSub `MathUtil.clamp()` return value was being discarded — extension was never actually clamped
- Fixed hub coordinates from (8.23, 8.23) to correct 2026 field position (4.63, 4.03)

### Calibration status
- Limelight pose fusion: working
- Shooter lookup table: placeholder values, needs real calibration data
- See `TESTING_PLAN.md` for calibration steps

### New files
- `CLAUDE.md` — project context and architecture reference
- `DEPLOY_NOTES.md` — build, deploy, and tooling setup instructions
- `TESTING_PLAN.md` — step-by-step Limelight validation and shooter calibration guide







| 🧑‍💻 DEV | 🐞 DEBUG | 🏆 STABLE |
| :--- | :--- | :--- |
| **Development** | **Debug** | **Stable ready for merge into main** |
| `Branch: dev` | `Branch: debug` | `Branch: main/stable` |
| New Subsystems | Sensor Calibration | Competition Approved |
| Experimental PID | Driver Practice | Verified Autos |



---

## 🏎️ The Workflow

### 🧑‍💻 `dev` — The Development Of Subsystems
*   **When to use:** Use this for writing new commands, subsystems, or testing experimental autonomous paths.
*   **Risk:** High. The robot might exhibit unexpected behavior. 
*   **Rule:** Always keep the robot on a **tether** or a **block** when running `dev` code.

### 🐞 `debug` — The Debuging Phrase Before Implementation Into Main Branch
*   **When to use:** Use this during practice matches or in the pits to tune constants (PID, Gains, Offsets, FeedForward).
*   **Process:** Merge `dev` into `debug` once a feature "mostly works."
*   **Goal:** Iron out the bugs and ensure [AdvantageScope](https://github.com) or Shuffleboard logs look clean.

### 🏆 `stable` — Competition Gold
*   **When to use:** **ON THE FIELD.** This is the only branch deployed for Qualification and Playoff matches.
*   **Rule:** No code is merged here unless it has passed a full 2-minute "mock match" without crashing. Stable code is then Merged into main branch after passing pull request from either the lead programmer or                 mentor
*   **Deployment:** Use the [WPILib VS Code extension](https://docs.wpilib.org) to deploy from this branch only.

---

## 🔄 Deployment Logic

```mermaid
graph TD
    A[New Feature / Subsystem] -->|Push| B(dev)
    B -->|Passed Bench Test| C(debug)
    C -->|Tuning & Bug Fixes| C
    C -->|Driver Approved| D{stable}
    D -->|Merge to Main| E[MATCH PLAY]
    
    style B fill:#3498db,stroke:#333,stroke-width:2px,color:#fff
    style C fill:#f39c12,stroke:#333,stroke-width:2px,color:#fff
    style D fill:#27ae60,stroke:#333,stroke-width:4px,color:#fff
    style E fill:#c0392b,stroke:#333,stroke-width:2px,color:#fff
    style A fill:#19ab11,stroke:#333,stroke-width:2px
    style B fill:#3498db,stroke:#fff,stroke-width:2px,color:#fff
    style C fill:#e67e22,stroke:#fff,stroke-width:2px,color:#fff
    style D fill:#2ecc71,stroke:#fff,stroke-width:2px,color:#fff


