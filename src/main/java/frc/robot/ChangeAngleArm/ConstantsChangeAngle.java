package frc.robot.ChangeAngleArm;

import frc.demacia.utils.Sensors.LimitSwitchConfig;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public class ConstantsChangeAngle {
    public static final double GEAR_RATIO = 1.0 / 175.0;
    public static final int MOTOR_ID = 1;
    public static final double MAX_VELOCITY = 0.75 * Math.PI;
    public static final double MAX_ACCEL = 2 * MAX_VELOCITY;
    public static final double MAX_JERK = 2 * MAX_ACCEL;
    public static final Canbus CANBUS_NAME = Canbus.CANIvore;
    public static final double KP = -1;
    public static final double KI = -1;
    public static final double KD = -1;
    public static final double KS = -1;
    public static final double KV = -1;
    public static final double KA = -1;
    public static final double KG = -1;  //Maybe change the final to not final

    public static final int LIMIT_SWITCH_CHANNEL = 0;

    public static final TalonConfig CHANGE_ANGLE_CONFIG =  //Maybe change the final to not final
        new TalonConfig(MOTOR_ID, CANBUS_NAME, "Change Angle Motor")
        .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK)
        .withRadiansMotor(GEAR_RATIO)
        .withPID(KP, KI, KD, KS, KV, KA, KG);
    public static final LimitSwitchConfig CHANGE_ANGLE_LIMIT_CONFIG = 
        new LimitSwitchConfig(LIMIT_SWITCH_CHANNEL, "Change Angle Limit Switch");
}
