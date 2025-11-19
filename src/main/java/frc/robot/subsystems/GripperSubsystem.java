// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.UltraSonicSensor;
import frc.robot.GripperConstants;
public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  private final TalonMotor motor;
  private final UltraSonicSensor ultrasonicSensor;

  public GripperSubsystem() {
    motor = new TalonMotor(GripperConstants.TALON_CONFIG);
    ultrasonicSensor = new UltraSonicSensor(GripperConstants.ULTRA_SONIC_SENSOR_CONFIG);
  }
  public double getRange() {
return ultrasonicSensor.getRangeMeters();
  }
    

    public boolean isCoralIn() {
      if (getRange() <0.10 ) return true;
      else return false;
    }
      public boolean isCubeIn() {
        if (getRange() <0.10 ) return true;
        else return false;
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}