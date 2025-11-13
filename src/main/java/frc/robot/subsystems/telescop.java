// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

public class telescop extends SubsystemBase {
  /** Creates a new telescop. */
  public telescop() {}

  private TalonMotor telescopMotor;

  private void subsystemsTelescope(){
    TalonConfig config = new TalonConfig(0, Canbus.Rio, "telescopMotor");
    
    telescopMotor = new TalonMotor(config);
  }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
