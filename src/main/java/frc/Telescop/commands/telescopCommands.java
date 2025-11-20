// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Telescop.commands;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.Telescop.ConstantsTelescop.STATE;

import frc.Telescop.ConstantsTelescop.STATE;

import frc.Telescop.ConstantsTelescop.STATE;
import frc.Telescop.subsystems.Telescop;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TelescopCommands extends Command {
  /** Creates a new telescopCommands. */

  private Telescop SubSystem; 
  
  STATE currentState = STATE.HOME;
    
  public TelescopCommands(Telescop subSystem) {
    this.SubSystem = subSystem;
    addRequirements(subSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SubSystem.startPozesan();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentState) {
      case L1, L2, L3, L4, peckUp, HOME, IDLE, TESTING :
      SubSystem.extendTelescope(currentState.length);
        break;
      case close:
        SubSystem.extendTelescope(currentState.length);
        SubSystem.rsetEncoder();
      case open:
        SubSystem.open();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SubSystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
