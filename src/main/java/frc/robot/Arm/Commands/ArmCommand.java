// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm.Commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Arm.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommand extends Command {
  Arm arm;

  double testAngle = 0;
  double testPosition = 0;

  public ArmCommand(Arm arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    testAngle = arm.getAngleMotor();
    testPosition = arm.getCurrentHeigt();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!arm.isCalibreated()) {
      arm.stop();
      return;
    }

    switch (arm.getCurrentState()) {
      case IDLE:
        arm.setAngle(arm.getAngleMotor());
        arm.setLength(arm.getCurrentHeigt());
        break;

      case TESTING:
        arm.setAngle(testAngle);
        arm.setLength(testPosition);
        break;
      default:

        arm.setAngle(arm.getCurrentState().angle);
        arm.setLength(arm.getCurrentState().length);
        break;
    }

  }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


  public double getTestAngle() {
    return testAngle;
  }

  public void setTestAngle(double testAngle) {
    this.testAngle = testAngle;
  }

  public double getTestPosition() {
    return testPosition;
  }

  public void setTestPosition(double testPosition) {
    this.testPosition = testPosition;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      super.initSendable(builder);
      builder.addDoubleProperty("testPzsan",this::getTestPosition , this::setTestPosition);
      builder.addDoubleProperty("testAngle",this::getTestAngle , this::setTestAngle);
  }
  
  public boolean isFinished() {
    return false;
  }
}
