// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.demacia.utils.Controller.CommandController;
import frc.demacia.utils.Controller.CommandController.ControllerType;
import frc.demacia.utils.Log.LogManager;
// import frc.robot.ChangeAngleArm.commands.GoToTelescopeAngle;
import frc.robot.ChangeAngleArm.commands.ManualControlAngleArm;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Gripper.subsystems.GripperSubsystem;
import frc.robot.Telescop.subsystems.TelescopSubSystem;
import frc.robot.Telescop.commands.CalibrationCommand;
import frc.robot.Telescop.commands.CalibrationCommands;
import frc.robot.Telescop.commands.TelescopCommands;
import frc.robot.Telescop.commands.ControllerTelescope;
import frc.robot.ChangeAngleArm.subsystems.ChangeAngle;
import frc.robot.Gripper.subsystems.GripperSubsystem;
import frc.robot.Gripper.commands.GripperCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static boolean isComp = DriverStation.isFMSAttached();
  private static boolean hasRemovedFromLog = false;

  public static int N_CYCLE = 0;
  public static double CYCLE_TIME = 0.02;

  // The robot's subsystems and commands are defined here...
  public static TelescopSubSystem TelescopSubSystem;
  public static TelescopCommands TelescopCommands;
  public static CalibrationCommands CalibrationCommands;
  public static CommandController controller;
  public static ChangeAngle changeAngle;
  public static GripperSubsystem gripperSubsystem;
  public static GripperCommand GripperCommand;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    new LogManager();
    changeAngle = new ChangeAngle();

    TelescopSubSystem = new TelescopSubSystem();
    // gripperSubsystem = new GripperSubsystem();
    controller = new CommandController(Constants.ControllerPort, ControllerType.kPS5);
    TelescopCommands = new TelescopCommands(TelescopSubSystem);
    TelescopSubSystem.setDefaultCommand(TelescopCommands);
    gripperSubsystem.setDefaultCommand(GripperCommand);
    // changeAngle.setDefaultCommand(new GoToTelescopeAngle(changeAngle));
    configureBindings();
  }

  public static boolean isComp() {
    return isComp;
  }

  public static void setIsComp(boolean isComp) {
    RobotContainer.isComp = isComp;
    if (!hasRemovedFromLog && isComp) {
      hasRemovedFromLog = true;
      LogManager.removeInComp();
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    controller.getRightStickMove().onTrue(new ControllerTelescope(controller, TelescopSubSystem));
    controller.getLeftStickMove().onTrue(new ManualControlAngleArm(changeAngle, controller));
    controller.downButton().onTrue(new CalibrationCommand(TelescopSubSystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
