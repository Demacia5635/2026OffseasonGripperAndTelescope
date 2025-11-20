// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.util.sendable.SendableBuilder;
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
      if (getRange() <= 0.03 ) return true;
      else return false;
    }
    public boolean isCubeIn() {
      if (!isCoralIn() && getRange() <=0.3) return true;
       else return false;
    }

    public void setPower(double power){
      motor.setDuty(power);
    }

    public void stop (){
      motor.stopMotor();
    }

    public void getCoral(){
      while (!isCoralIn()) {
          setPower(0.3);
        }
        stop();
    }
    public void getCube(){
      while (!isCubeIn()) {
          setPower(0.3);
        }
        stop();
    }
    public void Out(){
      while (isCoralIn()|| isCubeIn()) {
          setPower(-0.3);
        }
        stop();
    }
  
  
  
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Is Coral In", ()-> isCoralIn(), null);
    builder.addBooleanProperty("Is Cube In", ()-> isCubeIn(), null);
    builder.addDoubleProperty("Get Range", ()-> getRange(), null);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}