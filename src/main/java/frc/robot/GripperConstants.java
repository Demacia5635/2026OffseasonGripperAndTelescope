package frc.robot;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Sensors.UltraSonicSensorConfig;
public class GripperConstants {
    public static final int echoChannel = 0;
    public static final int pingChannel = 0;
    public static final int motor_ID = 0;
    public static final Canbus CANBUS = Canbus.CANIvore;
    public static final double MAX_VELOCITY = 3;
    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_JERK = 0;
    public static final double DIAMETER = 0;
    public static final double GEAR_RATIO = 0;
    public static final double MAX_CURRENT = 0;    

    public static final UltraSonicSensorConfig ULTRA_SONIC_SENSOR_CONFIG = new UltraSonicSensorConfig(echoChannel, pingChannel, null);

    public static final TalonConfig TALON_CONFIG = new TalonConfig(motor_ID, CANBUS, null)
    .withBrake(false)
    .withMeterMotor(GEAR_RATIO, DIAMETER)
    .withMotionParam(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
    .withCurrent(MAX_CURRENT);



}
