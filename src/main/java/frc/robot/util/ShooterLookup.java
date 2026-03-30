package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookup {

    // Distance (feet) -> Shooter velocity (FPS)
    // TODO: Replace with real calibration data
    private static final TreeMap<Double, Double> table = new TreeMap<>();

    static {
        table.put(3.0,  45.0);
        table.put(5.0,  55.0);
        table.put(7.0,  65.0);
        table.put(9.0,  75.0);
        table.put(12.0, 90.0);
    }

    public static double getInterpolatedVelocity(double distanceFt) {
        Map.Entry<Double, Double> lower = table.floorEntry(distanceFt);
        Map.Entry<Double, Double> upper = table.ceilingEntry(distanceFt);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue();

        double t = (distanceFt - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lower.getValue() + (upper.getValue() - lower.getValue()) * t;
    }
}
