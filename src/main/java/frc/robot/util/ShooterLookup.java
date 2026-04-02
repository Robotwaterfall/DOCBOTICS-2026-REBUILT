package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookup {

    // Distance (feet) -> Shooter velocity (FPS)
    // TODO: Replace with real calibration data
    private static final TreeMap<Double, Double> table = new TreeMap<>();

    static {
        table.put(4.25,  40.0);
        table.put(5.0,  50.0);
        table.put(7.0,  54.0);
        table.put(9.0,  57.0);
        table.put(12.0, 60.0);
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
