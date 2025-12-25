package frc.robot.ChangeAngleArm;

import frc.demacia.utils.Sensors.AnalogEncoderConfig;

import frc.demacia.utils.Motors.TalonFXConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public class ConstantsChangeAngle {
    public static final double GEAR_RATIO = 12.8;
    public static final int MOTOR_ID = 11;
    public static final double MAX_VELOCITY = 5 * Math.PI;
    public static final double MAX_ACCEL = 5 * MAX_VELOCITY;
    public static final double MAX_JERK = 5 * MAX_ACCEL;
    public static final Canbus CANBUS_NAME = Canbus.Rio;
    public static final double KP = 2;
    public static final double KI = 0;
    public static final double KD = 0.02;
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double KG = 0; // Maybe change the final to not final

    public static final int ANALOG_ENCODER_CHANNEL = 1;

    public static final TalonFXConfig CHANGE_ANGLE_CONFIG = // Maybe change the final to not final
            new TalonFXConfig(MOTOR_ID, CANBUS_NAME, "Change Angle Motor")
                    .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK)
                    .withRadiansMotor(GEAR_RATIO)
                    .withPID(KP, KI, KD, KS, KV, KA, KG);

    public static final AnalogEncoderConfig CHANGE_ANGLE_ANALOG_CONFIG = new AnalogEncoderConfig(ANALOG_ENCODER_CHANNEL,
            "Change Angle Encoder");

    public static enum TELESCOPE_ANGLE {
        IDLE("Idle", 0, 0),
        IDLE2("Idle2", 0, 0),
        TOP("Top", Math.toRadians(90), 0);

        public final double angle;
        public final double length;
        public final String name;

        TELESCOPE_ANGLE(String name, double angle, double length) {
            this.angle = angle;
            this.length = length;
            this.name = name;
        }
    }
}
