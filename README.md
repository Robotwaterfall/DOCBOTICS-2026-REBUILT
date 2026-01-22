
  
  # 🤖 Docbotics 6110 Code Pipeline







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


