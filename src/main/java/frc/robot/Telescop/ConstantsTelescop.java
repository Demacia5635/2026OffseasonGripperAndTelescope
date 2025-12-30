package frc.robot.Telescop;

import frc.demacia.utils.Motors.TalonFXConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Sensors.LimitSwitchConfig;

public class ConstantsTelescop {


    public static final LimitSwitchConfig SENSOR_CONFIG = new LimitSwitchConfig(3, "Telescop Sensor").withInvert(true);

    public static enum STATE_TELESCOPE {
        OUT_TAKE(0,0),
        HOME(0, 0.05),
        TESTING(-1, -1),
        IDLE(-1, -1),
        INTAKE(0, 0),
        CLOSED(-1, 0),
        OPEN(-1, 0.40),
        CALIBRATE(0, 0.03);

        public double angle;
        public double length;

        STATE_TELESCOPE(double angle, double length) {
            this.angle = angle;
            this.length = length;
        }
    }

    public static final double MAX_LENGTH = 0.7;
    public static final double MIN_LENGTH = 0;

    public static final double MAX_VELOCITY = 1.2;
    public static final double MAX_ACCELERATION = 6;
    public static final double MAX_JERK = 0;

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