// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Motors.TalonFXConfig;
import frc.demacia.utils.Sensors.DigitalEncoderConfig;
import frc.demacia.utils.Sensors.LimitSwitchConfig;

/** Add your docs here. */
public class ArmConstants {
    public static enum STATE {

        HOME(0, 0.05,0),
        TESTING(-1, -1,0),
        IDLE(-1, -1,0),
        LOW_CLOSED(0, 0.05,0),
        LOW_OPEN(0, 0.6,0),
        MID_CLOSED(Math.toRadians(30), 0.05,0),
        MID_OPEN(Math.toRadians(30), 0.6,0),
        HIGH_CLOSED(Math.toRadians(60), 0.05,0),
        HIGH_OPEN(Math.toRadians(60), 0.6,0),
        GET_CORAL(Math.toRadians(0), 0,0.3),
        GET_CUBE(Math.toRadians(0), 0,0.3);
        EJECT(Math.toRadians(0), 0, );

        public double angle;
        public double length;
        public double duty;

        STATE(double angle, double length, double duty) {
            this.angle = angle;
            this.length = length;
            this.duty = duty;
        }
    }

    public class TelescopConstants {

        public static final LimitSwitchConfig SENSOR_CONFIG = new LimitSwitchConfig(3, "Telescop Sensor")
                .withInvert(true);

        public static final double MAX_LENGTH = 0.7;
        public static final double MIN_LENGTH = 0;

        public static final double MAX_VELOCITY = 1.2;
        public static final double MAX_ACCELERATION = 6;
        public static final double MAX_JERK = 11;

        public static final int ID = 40;
        public static final Canbus CANBUS = Canbus.Rio;
        public static final double GEAR_RATIO = 58.24 / 0.2268; // gear ratio correct
        public static final double DIAMETER = 1;
        public static final double MAX_CURRENT = 40;

        public static final double kp = 2;
        public static final double ki = 0;
        public static final double kd = 0;
        public static final double ks = 0.14;
        public static final double kv = 9.3;
        public static final double ka = 0.00028;
        public static final double kg = 0;
        public static final double kg2 = 0;
        public static final int UP_CHANNEL = -1;
        public static final int DOWN_CHANNEL = -1;
        public static final double angle = 0;

        public static final TalonFXConfig MOTOR_CONFIG = new TalonFXConfig(ID, CANBUS, "telescopMotor")
                .withBrake(true)
                .withMeterMotor(GEAR_RATIO, DIAMETER)
                .withPID(kp, ki, kd, ks, kv, ka, kg)
                .withMotionParam(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
                .withCurrent(MAX_CURRENT);

        public static class CalibrationConstants {
            public static final double TIME_TO_MOVE_CALIBRATION = 0.5; // seconds
            public static final double POSITION_AT_BOTTOM_SWITCH = 0;
            public static final double POWER_AT_BOTTOM_SWITCH = 0;
            public static final double POWER_TO_TOP = 0.2;
            public static final double POWER_TO_BOTTOM = -0.1;
            public static final double POWER_UP_AT_START = 0.05;

        }
    }

    public static class AngleChangeConstants {
        public static final LimitSwitchConfig SENSOR_CONFIG_ANGLE = new LimitSwitchConfig(2,
                "Angle change limit switch").withInvert(true);

        public static final double GEAR_RATIO = 64 * 42 / 22.0;
        public static final int MOTOR_ID = 11;
        public static final double MAX_VELOCITY = 2;
        public static final double MAX_ACCEL = 8;
        public static final double MAX_JERK = 20;
        public static final Canbus CANBUS_NAME = Canbus.Rio;
        public static final double KP = 2;
        public static final double KI = 0;
        public static final double KD = 0.02;
        public static final double KS = 0.142578125;
        public static final double KV = 2.3459699153900146;
        public static final double KA = 0;
        public static final double KG = 0.03516;

        public static final int ANALOG_ENCODER_CHANNEL = 0;

        public static final TalonFXConfig CHANGE_ANGLE_CONFIG = new TalonFXConfig(MOTOR_ID, CANBUS_NAME,
                "Change Angle Motor")
                .withMotionParam(MAX_VELOCITY, MAX_ACCEL, MAX_JERK)
                .withRadiansMotor(GEAR_RATIO)
                .withPID(KP, KI, KD, KS, KV, KA, 0)
                .withInvert(true);

        public static final DigitalEncoderConfig CHANGE_ANGLE_ANALOG_CONFIG = new DigitalEncoderConfig(
                ANALOG_ENCODER_CHANNEL,
                "Change Angle Encoder").withScalar(-1).withOffset(1);

    }
}
