    package frc.robot.Gripper;

    //import frc.demacia.utils.Motors.TalonConfig;
    import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.demacia.utils.Motors.TalonFXConfig;
import frc.demacia.utils.Sensors.UltraSonicSensorConfig;

    public class GripperConstants {

      public enum GRIPPER_STATE {
        IDLE(0),
        GET_CORAL(0.3),
        GET_CUBE(0.3),
        TESTING(0),
        EJECT(-0.3);

        public double duty;

        GRIPPER_STATE(double duty) {
          this.duty = duty;
        }
      }

      public static final double coralDetectedDistance = 0.03;
      public static final double cubeDetectedDistance = 0.3;
      public static final double holdCoralVoltage = 0.01;
      public static final int echoChannel = 0;
      public static final int pingChannel = 0;
      public static final int motor_ID = 0;
      public static final Canbus CANBUS = Canbus.CANIvore;
      public static final double MAX_CURRENT = 80;

      public static final UltraSonicSensorConfig ULTRA_SONIC_SENSOR_CONFIG = new UltraSonicSensorConfig(echoChannel,
          pingChannel, null);

      public static final TalonFXConfig TALON_CONFIG = new TalonFXConfig(motor_ID, CANBUS, null)
          .withBrake(false)
          .withCurrent(MAX_CURRENT)
          .withInvert(true);

    }
