// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Telescop.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Telescop.ConstantsTelescop;
import static frc.Telescop.ConstantsTelescop.STATE;
import frc.Telescop.subsystems.Telescop;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class telescopCommands extends Command {
  /** Creates a new telescopCommands. */

  Telescop telescopSubSystem;

  ConstantsTelescop constantsTelescop;
  
  STATE currentState = STATE.HOME;
    
  public telescopCommands() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (currentState) {
      case L1:
        telescopSubSystem.MoveToL1();
        break;
      case L2:
        telescopSubSystem.MoveToL2();
        break;
      case L3:
        telescopSubSystem.MoveToL3();
        break;
      case L4:
        telescopSubSystem.MoveTOL4();
        break;
      case peckUp:
        telescopSubSystem.moveToPeckUp();
      case HOME:
        telescopSubSystem.MoveToHOME();
        break;
      case IDLE:
        telescopSubSystem.MoveToIDLE();
        break;
      case TESTING:
        telescopSubSystem.MoveToTESTING();
        break;
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
