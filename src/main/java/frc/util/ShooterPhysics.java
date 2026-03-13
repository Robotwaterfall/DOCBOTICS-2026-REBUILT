/*
 * Co-Pilot generated to use for your inspiration... must provide proper implementation to function
 * May or may not work... ;)
 */

/*
 * ShooterPhysics
 *
 * This class models the projectile physics of an FRC shooter.
 * It computes the hood angle + exit velocity needed to score a shot
 * such that the ball enters the hub on a *downward* trajectory.
 *
 * IMPORTANT:
 * - All units are in INCHES and INCHES/SECOND.
 * - Gravity is converted to inches/s^2.
 * - This class contains NO WPILib code. It is pure math.
 *
 * Why separate physics from the subsystem?
 * - Easier to test
 * - Easier to tune
 * - Easier to reuse in commands
 * - No motor or hardware dependencies
 */

package frc.util;

public class ShooterPhysics {

    // Shooter exit height (inches)
    // This is the height where the ball leaves the flywheel.
    private final double shooterHeightIn;

    // Hub center height (inches)
    // This is the target height the projectile must reach.
    private final double hubHeightIn;

    // Gravity in inches per second squared.
    // 1 g = 386.088 in/s^2
    private final double g = 386.088;

    /*
     * Constructor stores the geometry of your robot + field target.
     * These values do not change during a match, so they belong here.
     */
    public ShooterPhysics(double shooterHeightIn, double hubHeightIn) {
        this.shooterHeightIn = shooterHeightIn;
        this.hubHeightIn = hubHeightIn;
    }

    /*
     * computeOptimalShot(distanceIn)
     *
     * Given a horizontal distance to the hub (in inches),
     * this function finds the BEST hood angle + exit velocity
     * that produces a DOWNWARD shot into the hub.
     *
     * Why sweep angles?
     * - There is no closed-form solution for "best angle that gives downward entry".
     * - Sweeping is extremely fast and reliable for FRC (only ~200 iterations).
     *
     * Optimization rule:
     * - Choose the HIGHEST valid angle.
     *   Higher angles produce softer arcs and more consistent shots.
     */
    public ShooterSolution computeOptimalShot(double distanceIn) {
        double bestAngle = Double.NaN;
        double bestVelocity = Double.NaN;

        // Vertical height difference between shooter and hub
        double h = hubHeightIn - shooterHeightIn;

        // Sweep hood angles from 20° to 65°
        // These limits match typical FRC hood mechanisms.
        for (double angleDeg = 20; angleDeg <= 65; angleDeg += 0.25) {

            // Convert degrees → radians for trig functions
            double theta = Math.toRadians(angleDeg);

            double cos = Math.cos(theta);
            double tan = Math.tan(theta);

            /*
             * Projectile equation rearranged to solve for velocity:
             *
             * v = sqrt( g * d^2 / (2 * cos^2(theta) * (d*tan(theta) - h)) )
             *
             * If denominator <= 0, the shot is physically impossible at this angle.
             */
            double denom = 2 * cos * cos * (distanceIn * tan - h);
            if (denom <= 0) continue;

            double v = Math.sqrt((g * distanceIn * distanceIn) / denom);

            /*
             * DOWNWARD TRAJECTORY CHECK
             *
             * We compute the vertical velocity at the moment the ball reaches
             * the horizontal distance of the hub.
             *
             * vy = v*sin(theta) - g * t
             *
             * If vy >= 0, the ball is still rising → reject this angle.
             */
            double t = distanceIn / (v * cos);  // time to reach hub plane
            double vy = v * Math.sin(theta) - g * t;

            if (vy >= 0) continue;  // not descending → invalid shot

            /*
             * If we reach this point:
             * - The shot hits the hub height
             * - The ball is descending
             * - The physics are valid
             *
             * Optimization rule:
             * - Keep the highest valid angle (most consistent shot)
             */
            bestAngle = theta;
            bestVelocity = v;
        }

        // Return the best angle + velocity pair found
        return new ShooterSolution(bestAngle, bestVelocity);
    }

    /*
     * velocityToRPM()
     *
     * Converts linear exit velocity (in/s) into flywheel RPM.
     *
     * Why?
     * - Motors run in RPM, not linear velocity.
     * - The flywheel surface speed determines ball exit speed.
     */
    public double velocityToRPM(double exitVelocityInPerSec, double wheelRadiusIn) {
        double circumferenceIn = 2 * Math.PI * wheelRadiusIn;
        double revPerSec = exitVelocityInPerSec / circumferenceIn;
        return revPerSec * 60.0;  // convert RPS → RPM
    }

    /*
     * ShooterSolution
     *
     * Simple container for:
     * - angleRad: hood angle in radians
     * - velocityInPerSec: required exit velocity in inches/second
     *
     * Why a class?
     * - Angle and velocity are inseparable.
     * - Commands can read both cleanly.
     * - Prevents mixing up values or using wrong indices.
     */
    public static class ShooterSolution {
        public final double angleRad;
        public final double velocityInPerSec;

        public ShooterSolution(double angleRad, double velocityInPerSec) {
            this.angleRad = angleRad;
            this.velocityInPerSec = velocityInPerSec;
        }
    }
}