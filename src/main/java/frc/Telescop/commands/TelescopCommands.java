// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Telescop.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Telescop.subsystems.Telescop.STATE;
import frc.Telescop.subsystems.Telescop;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TelescopCommands extends Command {
  /** Creates a new telescopCommands. */

  private Telescop telescop; 
  
  Timer timer = new Timer();

  public TelescopCommands(Telescop subSystem) {
    this.telescop = subSystem;
    addRequirements(subSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(telescop.currentState == STATE.CALIBRATE) {
      if (timer.get() < 0.5){
        telescop.setPower(0.2);
      }
      if (Telescop.limitSwitchDown.get() == false){
        telescop.setPower(-0.1);
      }
    } else {
      telescop.setLengthPosition(telescop.currentState.length);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescop.Stop();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
