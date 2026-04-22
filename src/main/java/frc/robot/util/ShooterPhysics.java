package frc.robot.util;

public class ShooterPhysics {

    private final double g = 9.80665; // m/s^2

    private double hubFrontXM;
    private double hubDepthM;
    private double hubWidthM;
    private double hubRimHeightM;

    private final double ballRadiusM = 0.075;

    private final double fixedAngleDeg = 45.0;

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
            double fixedAngleDeg
    ) {
        double dx = hubX - shooterX;
        double dy = hubY - shooterY;
        double horizontalDistM = Math.hypot(dx, dy);

        if (horizontalDistM <= 0) {
            return ShotResult.invalid(fixedAngleDeg, fixedAngleDeg);
        }

        double yawRad = Math.atan2(dy, dx);

        ShotResult base = computeOptionC(
                horizontalDistM,
                shooterZ,
                hubZ,
                wheelRadiusM,
                efficiency,
                maxRPM,
                fixedAngleDeg
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
     * @param fixedAngleDeg hood angle deg
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
            double fixedAngleDeg
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
                fixedAngleDeg
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
            double fixedAngleDeg
    ) {
        double dx = hubX - shooterX;
        double dy = hubY - shooterY;
        double horizontalDistM = Math.hypot(dx, dy);

        if (horizontalDistM <= 0) {
            return ShotResult.invalid(fixedAngleDeg, fixedAngleDeg);
        }

        double yawRad = Math.atan2(dy, dx);

        ShotResult base = computeOptionC(
                horizontalDistM,
                shooterZ,
                hubZ,
                wheelRadiusM,
                efficiency,
                maxRPM,
                fixedAngleDeg
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
            double fixedAngleDeg
    ) {
        distanceRimM = Math.max(distanceRimM, 0);
        shooterHeightM = Math.max(shooterHeightM, 0);
        hubRimHeightM = Math.max(hubRimHeightM, shooterHeightM);
        wheelRadiusM = Math.max(wheelRadiusM, 0.01);
        efficiency = Math.max(0.01, Math.min(efficiency, 1.0));
        maxRPM = Math.max(maxRPM, 1);

        double theta = Math.toRadians(fixedAngleDeg);
        double cos = Math.cos(theta);
        double tan = Math.tan(theta);

        double h = hubRimHeightM - shooterHeightM;

        double denom = 2 * cos * cos * (distanceRimM * tan - h);

        if (denom <= 0) {
            return ShotResult.invalid(fixedAngleDeg, fixedAngleDeg);
        }

        double v = Math.sqrt((g * distanceRimM * distanceRimM) / denom);
        double t = distanceRimM / (v * cos);

        double vx = v * cos;
        double vy = v * Math.sin(theta) - g * t;

        if (vy >= 0) {
            return ShotResult.invalid(fixedAngleDeg, fixedAngleDeg);
        }

        double rpm = velocityToRPM(v, wheelRadiusM, efficiency);

        if (rpm > maxRPM) {
            return ShotResult.invalid(fixedAngleDeg, fixedAngleDeg);
        }

        double[][] traj = new double[0][0];

        return new ShotResult(
                theta,
                v,
                rpm,
                t,
                true,
                traj,
                theta,
                theta,
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