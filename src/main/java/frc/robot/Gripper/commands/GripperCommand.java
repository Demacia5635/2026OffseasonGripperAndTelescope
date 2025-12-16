// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Log.LogManager;
import frc.robot.Gripper.GripperConstants;
import frc.robot.Gripper.GripperConstants.CubeIntakeState;
import frc.robot.Gripper.GripperConstants.GRIPPER_STATE;
import frc.robot.Gripper.subsystems.GripperSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperCommand extends Command {
  /** Creates a new GripperCommand. */
  private GripperSubsystem gripperSubsystem;
  private Timer ejectTimer;
  private boolean isEjecting;
  private static final double EJECT_EXTRA_TIME = 0.5;
  private static final double CURRENT_THRESHOLD = 6;
  private final int INTAKE_CYCLE_TO_STOP = 20;
  private double intakeCount = 0;

  private int currentSpikeCounter = 0;

  GRIPPER_STATE currentState = GRIPPER_STATE.IDLE;
  CubeIntakeState currentIntakeState = CubeIntakeState.FULL_INTAKE;

  public GripperCommand(GripperSubsystem gripperSubsystem) {
    this.gripperSubsystem = gripperSubsystem;
    addRequirements(gripperSubsystem);
    ejectTimer = new Timer();
    isEjecting = false;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isEjecting = false;
    ejectTimer.stop();
    ejectTimer.reset();
    currentSpikeCounter = 0;
  }

  boolean startedCubeIntake = false;

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

        handleCubeIntakeWithCurrent();
        break;

      case EJECT:
        if (gripperSubsystem.hasGamePiece() || gripperSubsystem.isOut()) {
          gripperSubsystem.setDuty(GRIPPER_STATE.EJECT.duty);
          isEjecting = true;
          ejectTimer.reset();
        } else {
          if (isEjecting) {
            ejectTimer.start();
            isEjecting = false;
          }

          if (ejectTimer.get() < EJECT_EXTRA_TIME) {
            gripperSubsystem.setDuty(GRIPPER_STATE.EJECT.duty);
          } else {
            gripperSubsystem.stop();
            gripperSubsystem.setState(GRIPPER_STATE.IDLE);
            ejectTimer.stop();
            ejectTimer.reset();
          }
        }
        break;

      case TESTING:
        gripperSubsystem.setDuty(gripperSubsystem.getTestValue());
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
  private void handleCubeIntakeWithCurrent() {
    double currentAmper = gripperSubsystem.getCurrentAmpers();
    LogManager.log("CUR AMPS" + currentAmper);

    // Increment counter if current exceeds threshold
    if (currentAmper > CURRENT_THRESHOLD) {
      currentSpikeCounter++;
    } else {
      currentSpikeCounter = 0; // Reset counter if current drops below threshold
    }

    // Check if current has been above threshold for 0.1 seconds (assuming 20ms cycles)
    if (currentSpikeCounter >= 3) { // 0.1 seconds / 0.02 seconds per cycle = 5 cycles
      gripperSubsystem.setState(GRIPPER_STATE.IDLE);
      gripperSubsystem.stop();
    } else {
      gripperSubsystem.setDuty(GRIPPER_STATE.GET_CUBE.duty);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    gripperSubsystem.stop();
    ejectTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
