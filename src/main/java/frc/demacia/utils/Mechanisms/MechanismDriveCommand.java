// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Mechanisms;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MechanismDriveCommand extends Command {
  BaseMechanism mechanism;
  String motorName;
  Supplier<Double> power;

  /** Creates a new DriveCommand. */
  public MechanismDriveCommand(BaseMechanism mechanism, String motorName, Supplier<Double> power) {
    this.mechanism = mechanism;
    this.motorName = motorName;
    this.power = power;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mechanism);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mechanism.setPower(motorName, power.get());
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
