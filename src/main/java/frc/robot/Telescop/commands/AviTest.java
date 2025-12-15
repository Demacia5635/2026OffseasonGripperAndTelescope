// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Telescop.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telescop.subsystems.TelescopSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AviTest extends Command {
  double wantedDistance = 0.5;
  TelescopSubSystem telescop;
  public AviTest(TelescopSubSystem telescop) {
    this.telescop = telescop;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescop.setPower(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescop.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return telescop.getCurrentLength() >= 0.5;
  }
}
