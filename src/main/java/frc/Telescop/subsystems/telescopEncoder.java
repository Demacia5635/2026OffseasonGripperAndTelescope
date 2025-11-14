// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telescop.ConstantsTelescop;
import frc.demacia.utils.Sensors.AnalogEncoder;
import frc.demacia.utils.Sensors.AnalogEncoderConfig;


public class telescopEncoder extends SubsystemBase {
  /** Creates a new telescopEncoder. */

  ConstantsTelescop constants = new ConstantsTelescop();

  AnalogEncoder telescopEncoder;
  private double offset;

  public telescopEncoder() {
    AnalogEncoderConfig config = new AnalogEncoderConfig(0, "telscopEncoder");

    telescopEncoder = new AnalogEncoder(config);
  }
  
  
  public double getEncoderPosition(){
    double EncoderPosition = telescopEncoder.get();
    return EncoderPosition;
  }

  public void resetEncoder(){}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
