// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Telescop.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telescop.subsystems.TelescopSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AviTest extends Command {

  TelescopSubSystem telescop;
  double d = 0.3;
  public AviTest(TelescopSubSystem telescop) {
    this.telescop = telescop;
    addRequirements(telescop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    telescop.setMotorPosition(0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescop.testPosition(d);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescop.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(telescop.getCurrentLength() - d) < 0.01;
  }
}
