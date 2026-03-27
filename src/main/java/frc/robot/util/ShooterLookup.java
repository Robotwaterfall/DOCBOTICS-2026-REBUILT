package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;
import frc.robot.Constants.ShooterConstants;

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
        table.put(3.0,  new ShooterParams(70.5, ShooterConstants.kShooterVelocityFps));
        table.put(3.5,  new ShooterParams(68.5, ShooterConstants.kShooterVelocityFps));
        table.put(4.0,  new ShooterParams(66.5, ShooterConstants.kShooterVelocityFps));
        table.put(4.5,  new ShooterParams(65.0, ShooterConstants.kShooterVelocityFps));
        table.put(5.0,  new ShooterParams(63.4, ShooterConstants.kShooterVelocityFps));
        table.put(5.5,  new ShooterParams(62.1, ShooterConstants.kShooterVelocityFps));
        table.put(6.0,  new ShooterParams(61.0, ShooterConstants.kShooterVelocityFps));
        table.put(6.5,  new ShooterParams(60.0, ShooterConstants.kShooterVelocityFps));
        table.put(7.0,  new ShooterParams(59.0, ShooterConstants.kShooterVelocityFps));
        table.put(7.5,  new ShooterParams(58.5, ShooterConstants.kShooterVelocityFps));
        table.put(8.0,  new ShooterParams(57.5, ShooterConstants.kShooterVelocityFps));
        table.put(8.5,  new ShooterParams(57.0, ShooterConstants.kShooterVelocityFps));
        table.put(9.0,  new ShooterParams(56.3, ShooterConstants.kShooterVelocityFps));
        table.put(9.5,  new ShooterParams(55.5, ShooterConstants.kShooterVelocityFps));
        table.put(10.0, new ShooterParams(55.0, ShooterConstants.kShooterVelocityFps));
        table.put(10.5, new ShooterParams(54.5, ShooterConstants.kShooterVelocityFps));
        table.put(11.0, new ShooterParams(54.0, ShooterConstants.kShooterVelocityFps));
        table.put(11.5, new ShooterParams(53.5, ShooterConstants.kShooterVelocityFps));
        table.put(12.0, new ShooterParams(53.0, ShooterConstants.kShooterVelocityFps));    }

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