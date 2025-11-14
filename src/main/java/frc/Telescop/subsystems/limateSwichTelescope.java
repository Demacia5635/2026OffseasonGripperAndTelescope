// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Telescop.subsystems;

import com.revrobotics.spark.config.LimitSwitchConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Sensors.LimitSwitch;
import frc.demacia.utils.Sensors.LidarSensorConfig;

public class limateSwichTelescope extends SubsystemBase {
  /** Creates a new limateSwichTelescope. */
  public limateSwichTelescope() {}

  LimitSwitch limateSwich;

  LimitSwitchConfig config = new LimitSwitchConfig();
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
