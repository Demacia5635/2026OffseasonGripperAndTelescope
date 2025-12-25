package frc.robot.ChangeAngleArm;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public class ConstantsChangeAngle {
    public static final double GEAR_RATIO = 64 * 40 / 22;
    public static final int MOTOR_ID = 11;
    public static final double MAX_VELOCITY = 5 * Math.PI;
    public static final double MAX_ACCEL = 5 * MAX_VELOCITY;
    public static final double MAX_JERK = 5 * MAX_ACCEL;
    public static final Canbus CANBUS_NAME = Canbus.Rio;
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KG = 0;
    

    public static final int ANALOG_ENCODER_CHANNEL = 0;
    public static final double ANALOG_ENCODER_OFFSET = 0.202981280074532 * 2 * Math.PI;

    public static final TalonConfig CHANGE_ANGLE_CONFIG = // Maybe change the final to not final
            new TalonConfig(MOTOR_ID, CANBUS_NAME, "Change Angle Motor")
                    .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK)
                    .withRadiansMotor(GEAR_RATIO)
                    .withPID(KP, KI, KD, KS, KV, KA, 0)
                    .withInvert(true);

    public static enum TELESCOPE_ANGLE {
        IDLE(0, 0),
        IDLE2(0, 0),
        TOP(Math.toRadians(90), 0),
        TESTING(0, 0);

        public final double angle;
        public final double length;

        TELESCOPE_ANGLE(double angle, double length) {
            this.angle = angle;
            this.length = length;
        }
    }
}
