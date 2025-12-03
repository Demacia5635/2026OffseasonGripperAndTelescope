// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Telescop.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telescop.ConstantsTelescop;
import frc.robot.Telescop.ConstantsTelescop.STATE;
import frc.robot.Telescop.subsystems.Telescop;

import static frc.robot.Telescop.ConstantsTelescop.STATE;
import static frc.robot.Telescop.ConstantsTelescop.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TelescopCommands extends Command {
  /** Creates a new telescopCommands. */

  private Telescop telescop;

  public TelescopCommands(Telescop subSystem) {
    this.telescop = subSystem;
    addRequirements(subSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    STATE currentState = telescop.getCurrentState();
    if(currentState == STATE.IDLE){
      telescop.setLengthPosition(telescop.getCurrentLength());
    }
    else{
      telescop.setLengthPosition(currentState.length);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
