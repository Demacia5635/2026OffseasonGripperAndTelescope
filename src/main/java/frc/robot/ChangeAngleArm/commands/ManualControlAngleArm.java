// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChangeAngleArm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Controller.CommandController;
import frc.robot.ChangeAngleArm.subsystems.ChangeAngle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualControlAngleArm extends Command {
  /** Creates a new ManualControlAngleArm. */
  	private CommandController controller;
	private ChangeAngle changeAngle;

  	public ManualControlAngleArm(ChangeAngle changeAngle, CommandController controller) {
    	this.controller = controller;
		this.changeAngle = changeAngle;

		addRequirements(changeAngle);
    // Use addRequirements() here to declare subsystem dependencies.
  	}

  // Called when the command is initially scheduled.
  	@Override
	public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
    	double joyY = controller.getRightY();
		changeAngle.setPower(Math.pow(joyY, 2) / 4);
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
