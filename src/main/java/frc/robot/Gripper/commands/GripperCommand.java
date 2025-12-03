// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Gripper.GripperConstants;
import frc.robot.Gripper.GripperConstants.GRIPPER_STATE;
import frc.robot.Gripper.subsystems.GripperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperCommand extends Command {
  /** Creates a new GripperCommand. */
  private GripperSubsystem gripperSubsystem;

  GRIPPER_STATE currentState = GRIPPER_STATE.IDLE;

  public GripperCommand(GripperSubsystem gripperSubsystem) {
    this.gripperSubsystem = gripperSubsystem;
    addRequirements(gripperSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (gripperSubsystem.getState()) {
      case GET_CORAL:
        if (gripperSubsystem.isCoralIn()) {
          gripperSubsystem.setVoltage(GripperConstants.holdCoralVoltage);
        } else {
          gripperSubsystem.setDuty(GRIPPER_STATE.GET_CORAL.duty);
        }
        break;

      case GET_CUBE:
        if (gripperSubsystem.isCubeIn()) {
          gripperSubsystem.stop();
          gripperSubsystem.setState(GRIPPER_STATE.IDLE);
        } else {
          gripperSubsystem.setDuty(GRIPPER_STATE.GET_CUBE.duty);
        }
        break;

      case EJECT:
        if (gripperSubsystem.hasGamePiece()) {
          gripperSubsystem.setDuty(GRIPPER_STATE.EJECT.duty);
        } else {
          gripperSubsystem.stop();
          gripperSubsystem.setState(GRIPPER_STATE.IDLE);
        }
        break;

      case TESTING:
        gripperSubsystem.setDuty(gripperSubsystem.getState().duty);
        break;

      case IDLE:
        gripperSubsystem.stop();
        break;

      default:
        gripperSubsystem.setState(GRIPPER_STATE.IDLE);
        gripperSubsystem.stop();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
