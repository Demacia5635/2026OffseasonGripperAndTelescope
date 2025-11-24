// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Telescop.commands;

import java.time.format.SignStyle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.Telescop.subsystems.Telescop.STATE;
import frc.Telescop.subsystems.Telescop;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TelescopCommands extends Command {
  /** Creates a new telescopCommands. */

  private Telescop telescop; 
  private boolean calibrateUp = true;
  
  
  public TelescopCommands(Telescop subSystem) {
    this.telescop = subSystem;
    addRequirements(subSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    if(telescop.currentState == STATE.CALIBRATE) {
      if(calibrateUp && telescop.getLength() < STATE.CALIBRATE.length) {
        telescop.setPower(0);
      } else {
        calibrateUp = false;
        telescop.setPower(0);
      } 
    } 
    else if(telescop.currentState == STATE.IDLE){
      telescop.setLengthPosition(STATE.IDLE.length);
    }
    else {
      telescop.setLengthPosition(telescop.currentState.length);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescop.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
