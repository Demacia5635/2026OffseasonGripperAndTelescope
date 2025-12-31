// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not putcu anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int ControllerPort = 0;
  
  public Canbus roborioId;

  public static enum STATE{
    HOME(Math.toRadians(0),0,0),
    TESTING(Math.toRadians(0),0,0),
    IDLE(Math.toRadians(0),0,0),
    EJECT(Math.toRadians(0),0,-0.3),
    GET_CORAL(Math.toRadians(0),0,0.3),
    GET_CUBE(Math.toRadians(0),0,0.3),
    CLOSED(Math.toRadians(0),0,0),
    OPEN(Math.toRadians(0),0.50,0),
    CALIBRATE(Math.toRadians(0),0.03,0),
    TOP(Math.toRadians(90),0,0),
    MID(Math.toRadians(45),0,0),
    DOWN(Math.toRadians(0),0,0);

    public double angle;
    public double length;
    public double duty;

    STATE(double angle, double length, double duty){
      this.angle = angle;
      this.length = length;
      this.duty = duty;
    }
  }

}