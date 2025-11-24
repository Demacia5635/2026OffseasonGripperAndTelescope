package frc.Telescop;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Sensors.LimitSwitchConfig;

public class ConstantsTelescop {    


    public enum STATE{
        L1(0,0),
        L2(0,0),
        L3(0,0),
        L4(0,0),
        HOME(0,0),
        TESTING(-1, -1),
        IDLE(-1, -1),
        peckUp(0,0),
        close(-1,0),
        open(-1, -1);

        

        public double angle;
        public double length;
        STATE(double angle, double length){
            this.angle = angle;
            this.length = length;
        }
    }


    public static final double MAX_VELOCITY = 3;
    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_JERK = 0;

    public static final int ID = -1;
    public static final Canbus CANBUS = Canbus.CANIvore;
    public static final double GEAR_RATIO = 16; //mabe need to cange
    public static final double DIAMETER = 30;
    public static final double MAX_CURRENT = 12; 

    public static final double kp = -1; //TODO: CEAKE WHAT THE PID
    public static final double KI = 0;
    public static final double KD = 0;
    public static final double ks = -1; //TODO: CEAKE WHAT THE PID
    public static final double kv = -1; //TODO: CEAKE WHAT THE PID
    public static final double ka = -1; //TODO: CEAKE WHAT THE PID
    public static final double kg = -1; //TODO: CEAKE WHAT THE PID
    
    
    public static final int UP_CHANNEL = -1;
    public static final int DOWN_CHANNEL = -1;

    public static final TalonConfig MOTOR_CONFIG = new TalonConfig(ID, CANBUS, "telescopMotor")
    .withBrake(true)
    .withMeterMotor(GEAR_RATIO, DIAMETER)
    .withPID(kp, KI, KD, ks, kv, ka, kg)
    .withMotionParam(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
    .withCurrent(MAX_CURRENT);

    public static final LimitSwitchConfig CONFIG_UP = new LimitSwitchConfig(UP_CHANNEL, "up limit switch");
    
    public static final LimitSwitchConfig CONFIG_DOWN = new LimitSwitchConfig(DOWN_CHANNEL, "down limit switch");
}
