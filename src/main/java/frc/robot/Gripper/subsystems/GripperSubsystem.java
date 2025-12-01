// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Gripper.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.UltraSonicSensor;
import frc.robot.Gripper.GripperConstants.GRIPPER_STATE;
import frc.robot.Gripper.GripperConstants;

public class GripperSubsystem extends SubsystemBase {
  /** Creates a new GripperSubsystem. */
  private final TalonMotor motor;
  private final UltraSonicSensor ultrasonicSensor;

  private GRIPPER_STATE state;

  public GripperSubsystem() {
    motor = new TalonMotor(GripperConstants.TALON_CONFIG);
    ultrasonicSensor = new UltraSonicSensor(GripperConstants.ULTRA_SONIC_SENSOR_CONFIG);

    addNT();

    state = GRIPPER_STATE.IDLE;
  }

  public void addNT() {
    SendableChooser<GRIPPER_STATE> stateChooser = new SendableChooser<>();
    stateChooser.addOption("GET_CORAL", GRIPPER_STATE.GET_CORAL);
    stateChooser.addOption("GET_CUBE", GRIPPER_STATE.GET_CUBE);
    stateChooser.addOption("EJECT", GRIPPER_STATE.EJECT);
    stateChooser.addOption("IDLE", GRIPPER_STATE.IDLE);
    stateChooser.onChange(state -> this.state = state);
    SmartDashboard.putData(getName() + "Gripper State Chooser", stateChooser);

  }

  public double getRange() {
    return ultrasonicSensor.getRangeMeters();
  }

  public boolean isCoralIn() {
    return getRange() <= GripperConstants.coralDetectedDistance;
  }

  public boolean isCubeIn() {
    return !isCoralIn() && getRange() <= GripperConstants.cubeDetectedDistance;
  }

  public boolean hasGamePiece() {
    return isCubeIn() || isCoralIn();
  }

  public void setPower(double power) {
    motor.setDuty(power);
  }

  public void setVelocity(double velocity) {
    motor.setVelocity(velocity);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setState(GRIPPER_STATE state) {
    this.state = state;
  }

  public GRIPPER_STATE getState() {
    return state;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addBooleanProperty("Is Coral In", () -> isCoralIn(), null);
    builder.addBooleanProperty("Is Cube In", () -> isCubeIn(), null);
    builder.addBooleanProperty("Has Game Piece", () -> hasGamePiece(), null);
    builder.addDoubleProperty("Get Range", () -> getRange(), null);
  }

  @Override
  public void periodic() {
  }
}
