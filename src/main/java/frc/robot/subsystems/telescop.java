// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.TalonMotor;
import frc.ConstantsTelescop;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public class telescop extends SubsystemBase {

  ConstantsTelescop constants = new ConstantsTelescop();
  /** Creates a new telescop. */
  public telescop() {}

  private TalonMotor telescopMotor;
  

  private void subsystemsTelescope(){
    TalonConfig config = new TalonConfig(0, Canbus.Rio, "telescopMotor")
    .withBrake(false)
    .withVolts(12)
    .withDegreesMotor(14)
    .withPID(0, 0, 0, 0, 0, 0, 0)
    .withMotionParam(constants.getMaxVelocity(), constants.getMaxAcceleration(), constants.getMaxJerk());
    

    telescopMotor = new TalonMotor(config);
  }


  public void close(){
    telescopMotor.setMotion(0);
  }

  public void open(){
    telescopMotor.setMotion(1.35);
  }


  public double currentVelocity(){
    double currentVelocity = telescopMotor.getCurrentVelocity();
    return currentVelocity;
}



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
