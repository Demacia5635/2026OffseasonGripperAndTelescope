// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum GRIPPER_STATE{
        Idle(0),
        GET_CORAL(0.3),
        GET_CUBE(0.3),
        HAS_GAME_PIECE(0),
        EJECT(0.3);


      

        public double power;
        GRIPPER_STATE (double power){
          this.power = power;
        } 
    }





  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
