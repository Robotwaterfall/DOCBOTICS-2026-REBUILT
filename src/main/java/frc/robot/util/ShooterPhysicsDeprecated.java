package frc.robot.util;

public class ShooterPhysicsDeprecated {

    private final double g = 9.80665; // m/s^2

    private double hubFrontXM = 0;
    private double hubDepthM = 1.05918;
    private double hubWidthM = 1.1938;
    private double hubRimHeightM = 1.8288;

    private final double ballRadiusM = 0.075;

    public void setHubGeometry(double frontX, double rimHeight, double width, double depth) {
        this.hubFrontXM = frontX;
        this.hubRimHeightM = rimHeight;
        this.hubWidthM = width;
        this.hubDepthM = depth;
    }

    public ShotResult computeShotFromCoordinates(
            double shooterX,
            double shooterY,
            double shooterZ,
            double hubX,
            double hubY,
            double hubZ,
            double wheelRadiusM,
            double efficiency,
            double maxRPM,
            double minAngleDeg,
            double maxAngleDeg
    ) {
        double dx = hubX - shooterX;
        double dy = hubY - shooterY;
        double horizontalDistM = Math.hypot(dx, dy);
        if (horizontalDistM <= 0) {
            return ShotResult.invalid(minAngleDeg, maxAngleDeg);
     
        }

        double yawRad = Math.atan2(dy, dx);

        ShotResult base = computeOptionC(
                horizontalDistM,
                shooterZ,
                hubZ,
                wheelRadiusM,
                efficiency,
                maxRPM,
                minAngleDeg,
                maxAngleDeg
        );
        return new ShotResult(
                base.angleRad,
                base.velocityMPerSec,
                base.rpm,
                base.timeOfFlightSec,
                base.valid,
                base.trajectory,
                base.minAngleRad,
                base.maxAngleRad,
                yawRad
        );
    }

    /**
     * Robot-friendly API: compute a shot from values similar to the sliders.
     *
     * @param distanceToFrontHubM distance from shooter to front of hub (m)
     * @param shooterHeightM shooter height (m)
     * @param hubRimHeightM hub rim height (m)
     * @param hubWidthM hub width (m)
     * @param hubDepthM hub depth (m)
     * @param wheelRadiusM wheel radius (m)
     * @param efficiency drivetrain/wheel efficiency (0..1)
     * @param maxRPM maximum shooter RPM
     * @param minAngleDeg min hood angle deg
     * @param maxAngleDeg max hood angle deg
     * @return shot result
     */
    public ShotResult computeShot(
            double distanceToFrontHubM,
            double shooterHeightM,
            double hubRimHeightM,
            double hubWidthM,
            double hubDepthM,
            double wheelRadiusM,
            double efficiency,
            double maxRPM,
            double minAngleDeg,
            double maxAngleDeg
    ) {
        setHubGeometry(0, hubRimHeightM, hubWidthM, hubDepthM);
        double distanceRimM = Math.max(distanceToFrontHubM, 0) + hubDepthM / 2.0;
        return computeOptionC(
                distanceRimM,
                shooterHeightM,
                hubRimHeightM,
                wheelRadiusM,
                efficiency,
                maxRPM,
                minAngleDeg,
                maxAngleDeg
        );
    }

    public ShotResult computeShot3D(
            double shooterX,
            double shooterY,
            double shooterZ,
            double hubX,
            double hubY,
            double hubZ,
            double wheelRadiusM,
            double efficiency,
            double maxRPM,
            double minAngleDeg,
            double maxAngleDeg
    ) {
        double dx = hubX - shooterX;
        double dy = hubY - shooterY;
        double horizontalDistM = Math.hypot(dx, dy);
        if (horizontalDistM <= 0) {
            return ShotResult.invalid(minAngleDeg, maxAngleDeg);
        }
        double yawRad = Math.atan2(dy, dx);
        ShotResult base = computeOptionC(
                horizontalDistM,
                shooterZ,
                hubZ,
                wheelRadiusM,
                efficiency,
                maxRPM,
                minAngleDeg,
                maxAngleDeg
        );
        return new ShotResult(
                base.angleRad,
                base.velocityMPerSec,
                base.rpm,
                base.timeOfFlightSec,
                base.valid,
                base.trajectory,
                base.minAngleRad,
                base.maxAngleRad,
                yawRad
        );
    }

    public ShotResult computeOptionC(
            double distanceRimM,
            double shooterHeightM,
            double hubRimHeightM,
            double wheelRadiusM,
            double efficiency,
            double maxRPM,
            double minAngleDeg,
            double maxAngleDeg
    ) {
        distanceRimM = Math.max(distanceRimM, 0);
        shooterHeightM = Math.max(shooterHeightM, 0);
        hubRimHeightM = Math.max(hubRimHeightM, shooterHeightM);
        wheelRadiusM = Math.max(wheelRadiusM, 0.01);
        efficiency = Math.max(0.01, Math.min(efficiency, 1.0));
        maxRPM = Math.max(maxRPM, 1);

        double h = hubRimHeightM - shooterHeightM;

        double bestAngle = Double.NaN;
        double bestVelocity = Double.NaN;
        double bestTime = Double.NaN;
        double bestRPM = Double.NaN;
        double bestFinalAngle = Double.NaN;

        // Search from steepest → flattest
        for (double angleDeg = maxAngleDeg; angleDeg >= minAngleDeg; angleDeg -= 0.1) {

            double theta = Math.toRadians(angleDeg);
            double cos = Math.cos(theta);
            double tan = Math.tan(theta);

            double denom = 2 * cos * cos * (distanceRimM * tan - h);
            if (denom <= 0) continue;

            double v = Math.sqrt((g * distanceRimM * distanceRimM) / denom);
            double t = distanceRimM / (v * cos);

            double vx = v * cos;
            double vy = v * Math.sin(theta) - g * t;

            if (vy >= 0) continue;

            double rpm = velocityToRPM(v, wheelRadiusM, efficiency);

            // If RPM too high, lower hood (continue loop)
            if (rpm > maxRPM) continue;

            double finalAngle = Math.atan2(vy, vx); // negative = downward

            if (Double.isNaN(bestAngle)) {
                bestAngle = theta;
                bestVelocity = v;
                bestTime = t;
                bestRPM = rpm;
                bestFinalAngle = finalAngle;
            } else {
                if (finalAngle < bestFinalAngle - 1e-4) {
                    bestAngle = theta;
                    bestVelocity = v;
                    bestTime = t;
                    bestRPM = rpm;
                    bestFinalAngle = finalAngle;
                } else if (Math.abs(finalAngle - bestFinalAngle) <= 1e-4 && rpm < bestRPM) {
                    bestAngle = theta;
                    bestVelocity = v;
                    bestTime = t;
                    bestRPM = rpm;
                    bestFinalAngle = finalAngle;
                }
            }
        }

        if (Double.isNaN(bestAngle)) {
            return ShotResult.invalid(minAngleDeg, maxAngleDeg);
        }

    // We don't need the full trajectory for a stationary, hub-locked robot.
    // Return only the calculated angle and velocity (trajectory empty).
    double[][] traj = new double[0][0];

    return new ShotResult(
        bestAngle,
        bestVelocity,
        bestRPM,
        bestTime,
        true,
        traj,
        Math.toRadians(minAngleDeg),
        Math.toRadians(maxAngleDeg),
        0.0
    );
    }

    private double velocityToRPM(double ballVelocityMPerSec, double wheelRadiusM, double efficiency) {
        double wheelSurfaceVelocity = ballVelocityMPerSec / efficiency;
        double circumferenceM = 2 * Math.PI * wheelRadiusM;
        double revPerSec = wheelSurfaceVelocity / circumferenceM;
        return revPerSec * 60.0;
    }
    

    public static class ShotResult {
        public final double angleRad;
        public final double velocityMPerSec;
        public final double rpm;
        public final double timeOfFlightSec;
        public final boolean valid;
        public final double[][] trajectory;
        public final double minAngleRad;
        public final double maxAngleRad;
        public final double yawRad;

        public ShotResult(double angleRad,
                          double velocityMPerSec,
                          double rpm,
                          double timeOfFlightSec,
                          boolean valid,
                          double[][] trajectory,
                          double minAngleRad,
                          double maxAngleRad,
                          double yawRad) {
            this.angleRad = angleRad;
            this.velocityMPerSec = velocityMPerSec;
            this.rpm = rpm;
            this.timeOfFlightSec = timeOfFlightSec;
            this.valid = valid;
            this.trajectory = trajectory;
            this.minAngleRad = minAngleRad;
            this.maxAngleRad = maxAngleRad;
            this.yawRad = yawRad;
        }

        public static ShotResult invalid(double minAngleDeg, double maxAngleDeg) {
            return new ShotResult(
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    false, new double[0][0],
                    Math.toRadians(minAngleDeg),
                    Math.toRadians(maxAngleDeg),
                    0.0
            );
        }
    }
}