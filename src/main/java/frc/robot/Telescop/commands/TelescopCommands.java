// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Telescop.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Telescop.subsystems.TelescopSubSystem;
import static frc.robot.Telescop.ConstantsTelescop.STATE_TELESCOPE;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TelescopCommands extends Command {
  /** Creates a new telescopCommands. */

  private TelescopSubSystem telescop;

  public TelescopCommands(TelescopSubSystem subSystem) {
    this.telescop = subSystem;
    addRequirements(subSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Telescop Command Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!telescop.isCalibreated())
      return;

    STATE_TELESCOPE currentState = telescop.getCurrentState();
    switch (currentState) {
      case IDLE:
        telescop.setLength(telescop.getCurrentHeigt());
        break;
      case TESTING:
        double l = telescop.getTestingLength();
        telescop.setLength(l);
        break;
      default:
        telescop.setLength(currentState.length);
        break;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
