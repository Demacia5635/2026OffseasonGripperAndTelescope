// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ChangeAngleArm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Controller.CommandController;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
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
    // Use addRequirements() here to declare subsystem dependencies.
  	}

  // Called when the command is initially scheduled.
  	@SuppressWarnings("unchecked")
	@Override
	public void initialize() {
		LogManager.addEntry("wanted power", () -> joyY * 0.2).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).withIsSeparated(true).build();
	}

  // Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
    	joyY = -controller.getLeftY();
		//changeAngle.setPower(MathUtil.clamp(joyY * 0.2, 0.05, 1));
		changeAngle.setPower(joyY * 0.2);
	}

  // Called once the command ends or is interrupted.
	@Override
  	public void end(boolean interrupted) {
		changeAngle.setPower(0);
	}

  // Returns true when the command should end.
  	@Override
  	public boolean isFinished() {
    	return !controller.getLeftStickMove().getAsBoolean();
  	}
}
