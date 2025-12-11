// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Telescop.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.demacia.utils.Controller.CommandController;
import frc.demacia.utils.Log.LogManager;
import frc.robot.Telescop.subsystems.TelescopSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ControllerTelescope extends Command {
  /** Creates a new Controller. */
  
  private TelescopSubSystem telescop;
  private CommandController controller;

  private double joyX = controller.getLeftX();


  public ControllerTelescope(TelescopSubSystem telescop) {
    this.telescop = telescop;
    addRequirements(telescop);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(telescop.isCalibreated()){
      telescop.setPower(joyX * 0.5);
    }else{
      LogManager.log("not calberate");
      return;
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
