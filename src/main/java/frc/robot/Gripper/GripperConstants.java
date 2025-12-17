package frc.robot.Gripper;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Sensors.UltraSonicSensorConfig;

public class GripperConstants {
    
    public enum GRIPPER_STATE{
        IDLE(0),
        GET_CORAL(-0.7),
        GET_CUBE(-0.6),
        TESTING(0),
        EJECT(0.9);


      

        public double duty;
        GRIPPER_STATE (double duty){
          this.duty = duty;
        } 
    }

    public enum CubeIntakeState {
      FULL_INTAKE,
      ALIGN,
      DONE
    }
    public static final double coralDetectedDistance = 0.065;
    public static final double cubeDetectedDistance = 0.13;
    public static final double ejectOffset = 0.10;
    public static final double holdCoralVoltage = -0.04;
    public static final int echoChannel = 7;
    public static final int pingChannel = 8;
    public static final int motor_ID = 21;
    public static final Canbus CANBUS = Canbus.Rio;
    public static final double MAX_VELOCITY = 3;
    public static final double MAX_ACCELERATION = 0;
    public static final double MAX_JERK = 0;
    public static final double MAX_CURRENT = 40;    

    public static final UltraSonicSensorConfig ULTRA_SONIC_SENSOR_CONFIG = new UltraSonicSensorConfig(echoChannel, pingChannel, "sonic");

    public static final TalonConfig TALON_CONFIG = new TalonConfig(motor_ID, CANBUS, "motor")
    .withBrake(true)
    .withMotionParam(MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK)
    .withCurrent(MAX_CURRENT);



}
