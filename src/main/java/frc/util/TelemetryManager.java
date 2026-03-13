package frc.util;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelemetryManager {

    private final Map<String, Object> subsystems = new HashMap<>();
    private final double periodSeconds;
    private double lastUpdate = 0;

    public TelemetryManager(double periodSeconds){
        this.periodSeconds = periodSeconds;
    }

    public void registerSubsystem(String name, Object subsystem){
        subsystems.put(name, subsystem);
    }

    public void update() {
        double now = Timer.getFPGATimestamp();
            if (now - lastUpdate < periodSeconds) return;
                lastUpdate = now;

                for (var entry : subsystems.entrySet()) {
                    SmartDashboard.putString(entry.getKey(), entry.getValue().toString());
                }
    }


}
