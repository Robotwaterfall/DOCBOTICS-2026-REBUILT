package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

public class ShooterLookup {

    public static class ShooterParams {
        public final double angleDeg;
        public final double velocityFps;

        public ShooterParams(double angleDeg, double velocityFps) {
            this.angleDeg = angleDeg;
            this.velocityFps = velocityFps;
        }
    }

    private static final TreeMap<Double, ShooterParams> table = new TreeMap<>();

    /*
     * Place all known distance-to-shooter-params mappings here.
     * The getInterpolated() method will handle interpolation for in-between distances.
     * Distance (feet) -> (Angle (degrees), Velocity (feet per second))
     * 
     * TODO: Populate this table with real data from testing. The current values are placeholders and may not be accurate.
     */

    static {
        table.put(3.0,  new ShooterParams(70.5, 16.5));
        table.put(3.5,  new ShooterParams(68.5, 16.9));
        table.put(4.0,  new ShooterParams(66.5, 17.2));
        table.put(4.5,  new ShooterParams(65.0, 17.5));
        table.put(5.0,  new ShooterParams(63.4, 17.9));
        table.put(5.5,  new ShooterParams(62.1, 18.2));
        table.put(6.0,  new ShooterParams(61.0, 18.6));
        table.put(6.5,  new ShooterParams(60.0, 19.0));
        table.put(7.0,  new ShooterParams(59.0, 19.3));
        table.put(7.5,  new ShooterParams(58.5, 19.7));
        table.put(8.0,  new ShooterParams(57.5, 20.1));
        table.put(8.5,  new ShooterParams(57.0, 20.4));
        table.put(9.0,  new ShooterParams(56.3, 20.8));
        table.put(9.5,  new ShooterParams(55.5, 21.1));
        table.put(10.0, new ShooterParams(55.0, 21.5));
        table.put(10.5, new ShooterParams(54.5, 21.8));
        table.put(11.0, new ShooterParams(54.0, 22.2));
        table.put(11.5, new ShooterParams(53.5, 22.5));
        table.put(12.0, new ShooterParams(53.0, 22.9));
    }

    // Returns interpolated ShooterParams object for a given distance in feet
    public static ShooterParams getInterpolated(double distanceFt) {
        Map.Entry<Double, ShooterParams> lower = table.floorEntry(distanceFt);
        Map.Entry<Double, ShooterParams> upper = table.ceilingEntry(distanceFt);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();
        if (lower.getKey().equals(upper.getKey())) return lower.getValue();

        double d0 = lower.getKey();
        double d1 = upper.getKey();
        double t = (distanceFt - d0) / (d1 - d0);

        double angle = lerp(lower.getValue().angleDeg, upper.getValue().angleDeg, t);
        double velocity = lerp(lower.getValue().velocityFps, upper.getValue().velocityFps, t);

        return new ShooterParams(angle, velocity);
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}