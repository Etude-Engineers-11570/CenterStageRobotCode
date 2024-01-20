package org.firstinspires.ftc.teamcode.TeleOp;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

/**
 * Mecanum wheel drive calculations.
 * Input controls:
 *   V_d = desired robot speed.
 *   theta_d = desired robot velocity angle.
 *   V_theta = desired robot rotational speed.
 * Characteristic equations:
 *   V_{front,left} = V_d sin(theta_d + pi/4) + V_theta
 *   V_{front,right} = V_d cos(theta_d + pi/4) - V_theta
 *   V_{back,left} = V_d cos(theta_d + pi/4) + V_theta
 *   V_{back,right} = V_d sin(theta_d + pi/4) - V_theta
 */
public class Mecanum {
    /**
     * Mecanum motion vector.
     */
    public static class Motion {
        // Robot speed [-1, 1].
        public final double vD;
        // Robot angle while moving [0, 2pi].
        public final double thetaD;
        // Speed for changing direction [-1, 1].
        public final double vTheta;

        /**
         * Sets the motion to the given values.
         */

        //Stores the velcity
        public Motion(double vD, double thetaD, double vTheta) {
            this.vD = vD;
            this.thetaD = thetaD;
            this.vTheta = vTheta;
        }
    }

    /**
     * Gets the motion vector from the joystick values.
     * @param left_stick_x The left joystick X.
     * @param left_stick_y The left joystick Y.
     * @param right_stick_x The right joystick X.
     * @param right_stick_y The right joystick Y.
     * @return The Mecanum motion vector.
     */
    //Magnitude:
    public static Motion joystickToMotion(double left_stick_x,
                                          double left_stick_y,
                                          double right_stick_x,
                                          double right_stick_y) {
        //Velocity of driving:
        //It finds the hypotenuse of the left stick Y & X
        double vD = Math.sqrt(Math.pow(left_stick_x, 2) +
                Math.pow(left_stick_y, 2));

        //Finds the angle of driving:
        //It is in radiants, 0:0, 360:2pi. 0 is going right.
        double thetaD = Math.atan2(left_stick_x, left_stick_y);

        //Velocity of turning.
        double vTheta = -right_stick_x;

        //Calls Motion
        return new Motion(vD, thetaD, vTheta);
    }

    /**
     * Mecanum wheels, used to get individual motor powers.
     */
    public static class Wheels {
        // The mecanum wheels.
        public final double leftFront;
        public final double rightFront;
        public final double leftBack;
        public final double rightBack;

        /**
         * Sets the wheels to the given values.
         */
        public Wheels(double leftFront, double rightFront,
                      double leftBack, double rightBack) {
            this.leftFront = leftFront;
            this.rightFront = rightFront;
            this.leftBack = leftBack;
            this.rightBack = rightBack;
        }
    }

    /**
     * Gets the wheel powers corresponding to desired motion.
     * @param motion The Mecanum motion vector.
     * @return The wheels with clamped powers. [-1, 1]
     */
    public static Wheels motionToWheels(Motion motion) {
        //These are the 3 motions, again, that we use to figure out the powers
        double vD = motion.vD;
        double thetaD = motion.thetaD;
        double vTheta = motion.vTheta;

        //These are the 4 variables we reference while setting our powers
        //I added the Math.sqrt(2)
        double leftFront = Math.sqrt(2)*vD * Math.sin(thetaD + Math.PI / 4) + vTheta;
        double rightFront = Math.sqrt(2)*vD * Math.cos(thetaD + Math.PI / 4) + vTheta;
        double leftBack = Math.sqrt(2)*vD * Math.cos(thetaD + Math.PI / 4)  - vTheta;
        double rightBack = Math.sqrt(2)*vD * Math.sin(thetaD + Math.PI / 4) - vTheta;

        List<Double> motors = Arrays.asList(leftFront, rightFront,
                leftBack, rightBack);
        clampPowers(motors);
        return new Wheels(motors.get(0), motors.get(1), motors.get(2), motors.get(3));
    }

    /**
     * Clamps the motor powers while maintaining power ratios.
     * @param powers The motor powers to clamp.
     */
    private static void clampPowers(List<Double> powers) {
        double minPower = Collections.min(powers);
        double maxPower = Collections.max(powers);
        double maxMag = Math.max(Math.abs(minPower), Math.abs(maxPower));

        if (maxMag > 1.0) {
            for (int i = 0; i < powers.size(); i++) {
                powers.set(i, powers.get(i) / maxMag);
            }
        }
    }
}