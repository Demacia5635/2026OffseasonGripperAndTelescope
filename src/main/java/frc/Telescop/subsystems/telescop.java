// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Telescop.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telescop.ConstantsTelescop;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Telescop.Constants;

public class telescop extends SubsystemBase {

  ConstantsTelescop constants = new ConstantsTelescop();
  Constants mainConstants = new Constants();
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


  if (mainConstants.getState() == "open"){
    telescopMotor.setMotion(constants.open);
  }else if(mainConstants.getState() == "close"){
    telescopMotor.setMotion(constants.close);
  }else if(mainConstants.getState() == "L1"){
    telescopMotor.setMotion(constants.getL1());
  }else if(mainConstants.getState() == "L2"){
    telescopMotor.setMotion(constants.getL2());
  }else if(mainConstants.getState() == "L3"){
    telescopMotor.setMotion(constants.getL3());
  }else if(mainConstants.getState() == "L4"){
    telescopMotor.setMotion(constants.getL4());
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
