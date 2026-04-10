package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookup {

    // Distance (feet) -> Shooter velocity (FPS)
    private static final TreeMap<Double, Double> table = new TreeMap<>();

    // Tested on 2026-04-08 at DO'C in drama room
    static {
        table.put(5.0,  38.0);
        table.put(6.0,  40.0);
        table.put(8.5,  44.0);
        table.put(10.0,  48.0);
        table.put(11.5, 50.0);
        table.put(16.5, 58.0);
        table.put(40.0, 106.0);
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
