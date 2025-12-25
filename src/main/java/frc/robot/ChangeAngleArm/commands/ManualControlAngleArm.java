// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChangeAngleArm.commands;

import java.util.logging.LogManager;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Controller.CommandController;
import frc.demacia.utils.Log.LogManager2;
import frc.robot.ChangeAngleArm.subsystems.ChangeAngle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ManualControlAngleArm extends Command {

	double joyY = 0;
  /** Creates a new ManualControlAngleArm. */
  	private CommandController controller;
	private ChangeAngle changeAngle;

  	public ManualControlAngleArm(ChangeAngle changeAngle, CommandController controller) {
    	this.controller = controller;
		this.changeAngle = changeAngle;

		addRequirements(changeAngle);
		LogManager2.addEntry("Controller output", () -> joyY);
    // Use addRequirements() here to declare subsystem dependencies.
  	}


  // Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
    	joyY = controller.getLeftY()*0.5;
		changeAngle.setPower(joyY);
	}

	@Override
	public void end(boolean interrupted) {
		changeAngle.stop();
	}


  // Returns true when the command should end.
  	@Override
  	public boolean isFinished() {
    	return false;
  	}
}
