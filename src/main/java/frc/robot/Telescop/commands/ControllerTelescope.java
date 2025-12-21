// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Telescop.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Controller.CommandController;
import frc.robot.Telescop.subsystems.TelescopSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ControllerTelescope extends Command {
  CommandController controller;
  TelescopSubSystem telescop;
  public ControllerTelescope(CommandController controller, TelescopSubSystem telescop) {
    this.controller = controller;
    this.telescop = telescop;
    SmartDashboard.putData(this);
    addRequirements(telescop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    telescop.setPower(-0.3*Math.signum(controller.getLeftY()) *(Math.pow(controller.getLeftY(), 2)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("controller", ()->-controller.getLeftY() * 0.3, null);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
