// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Sensors.LimitSwitch;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  private final TalonFX leftmotor;
  private final TalonFX rightmotor;
  private final Ultrasonic ultrasonicSensor;
  private final LimitSwitch limitSwitch;
  
  public GripperSubsystem() {
    leftmotor = new TalonFX(0);
    rightmotor = new TalonFX(0);
    ultrasonicSensor = new Ultrasonic(0, 0);
    limitSwitch = new LimitSwitch(null);
    
  }
  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
