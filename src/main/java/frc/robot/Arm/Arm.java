// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Motors.TalonFXMotor;
import frc.demacia.utils.Sensors.DigitalEncoder;
import frc.demacia.utils.Sensors.LimitSwitch;
import frc.robot.Arm.ArmConstants.AngleChangeConstants;
import frc.robot.Arm.ArmConstants.STATE_TELESCOPE;
import frc.robot.Arm.ArmConstants.TelescopConstants;
import frc.robot.Arm.Commands.CalibrationCommand;

/** Add your docs here. */
public class Arm extends SubsystemBase {

    private LimitSwitch limitSwitchTelescope;
    private TalonFXMotor telescopMotor;
    private boolean calibrated = false;
    private STATE_TELESCOPE currentState = STATE_TELESCOPE.IDLE;

    DigitalEncoder changeAngleEncoder;
    TalonFXMotor changeAngleMotor;
    LimitSwitch limitSwitchChangeAngle;
    final double offset = -1.303466796875;

    /** Creates a new telescop. */
    public Arm() {
        limitSwitchTelescope = new LimitSwitch(TelescopConstants.SENSOR_CONFIG);
        telescopMotor = new TalonFXMotor(TelescopConstants.MOTOR_CONFIG);

        changeAngleMotor = new TalonFXMotor(AngleChangeConstants.CHANGE_ANGLE_CONFIG);
        limitSwitchChangeAngle = new LimitSwitch(AngleChangeConstants.SENSOR_CONFIG_ANGLE);
        changeAngleEncoder = new DigitalEncoder(AngleChangeConstants.CHANGE_ANGLE_ANALOG_CONFIG);
        changeAngleMotor.setEncoderPosition(changeAngleEncoder.get() - offset);
        putData();

        SmartDashboard.putData(getName() + "/set brake", new InstantCommand(()-> {
            telescopMotor.setNeutralMode(true);
            changeAngleMotor.setNeutralMode(true);
        }).ignoringDisable(true));

        SmartDashboard.putData(getName() + "/set coast", new InstantCommand(()-> {
            telescopMotor.setNeutralMode(false);
            changeAngleMotor.setNeutralMode(false);
            
        }).ignoringDisable(true));

        SmartDashboard.putData(getName() + "/Calibrate", new CalibrationCommand(this));

        SmartDashboard.putData("Telescop", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      
        builder.addBooleanProperty("Sensor", this::getLimitSensorTelescop, null);
        builder.addDoubleProperty("Length", this::getCurrentHeigt, null);
        builder.addStringProperty("State", () -> currentState.name(), null);
        builder.addBooleanProperty("Is at limit", () -> isAtBottom(), null);
        builder.addBooleanProperty("Is at Sensor Change Angle", () -> getLimitSensorAngle(), null);
        builder.addDoubleProperty("Angle Motor Target", () -> currentState.angle, null);
        builder.addDoubleProperty("Telescope Target", () -> currentState.length, null);
        // builder.addDoubleProperty("Test Length", () ->
        // STATE_TELESCOPE.TESTING.length, (l) -> STATE_TELESCOPE.TESTING.length = l);

    }

    @SuppressWarnings("unchecked")
    public void putData() {
        SendableChooser<STATE_TELESCOPE> stateChooser = new SendableChooser<>();
        for(STATE_TELESCOPE state : STATE_TELESCOPE.values()) {
            stateChooser.addOption(state.name(), state);
        }
        stateChooser.onChange(STATE -> setState(STATE));
        SmartDashboard.putData("Telescop State", stateChooser);
        LogManager.addEntry("Telescope", () -> (new double[] { getCurrentHeigt() }))
                .withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();

        LogManager.addEntry("Telescope2", () -> (new boolean[] { isAtBottom(), isCalibreated() }))
                .withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();

    }

    public double getCurrentHeigt() {
        return telescopMotor.getCurrentPosition();
    }

    public void setMotorLength(double l) {
        telescopMotor.setEncoderPosition(l);
    }

    public void stop() {
        telescopMotor.setDuty(0);
    }

    public void setPower(double power) {
        telescopMotor.setDuty(power);
    }

    public double getVelocity() {
        return telescopMotor.getCurrentVelocity();
    }

    public void testPosition(double pos) {
        telescopMotor.setPositionVoltage(pos);
    }

    public void setLength(double wantedLength) {
        if (!isCalibreated()) {
            LogManager.log("Telescop not calibrated");
            stop();
            return;
        }
        if (isAtBottom()) {
            LogManager.log("Telescop at limit switch");
            wantedLength = 0.03;

        }
        if (Math.abs(wantedLength - getCurrentHeigt()) <= 0.01) {
            stop();
            return;
        }
        telescopMotor
                .setMotion(MathUtil.clamp(wantedLength, TelescopConstants.MIN_LENGTH, TelescopConstants.MAX_LENGTH), 0);// Math.sin(RobotContainer.changeAngle.getAngle()));
    }
    public void setAnglePower(double power){
        if(getLimitSensorAngle()) stop();
        else{
            changeAngleMotor.set(power);
        }
    }

    public void setAngle(double angle) {
        if (angle < Math.toRadians(-30) || angle > Math.toRadians(90)) {
            changeAngleMotor.stop();
            return;

        }
        if (getLimitSensorAngle()) {
            angle = 0;
        }
        changeAngleMotor.setMotion(angle,
        AngleChangeConstants.KG * Math.cos(getAngleMotor()) * getCurrentHeigt() / TelescopConstants.MAX_LENGTH);
    }

    public double getAngleMotor() {
        return changeAngleMotor.getCurrentAngle();
    }

    public double getAngleSensor() {
        return changeAngleEncoder.get();
    }

    public boolean getLimitSensorAngle() {
        return limitSwitchChangeAngle.get();
    }

    public boolean isCalibreated() {
        return calibrated;
    }

    public void setCalibrated(boolean isCalibreated) {
        this.calibrated = isCalibreated;
    }

    public void setCalibrated() {
        setCalibrated(true);
    }

    public boolean isAtBottom() {
        return getLimitSensorTelescop();
    }

    public void setState(STATE_TELESCOPE state) {
        if (isCalibreated()) {
            currentState = state;
        } else {
            LogManager.log("not calibreated");
        }
    }

    public boolean getLimitSensorTelescop() {
        return limitSwitchTelescope.get();
    }

    public STATE_TELESCOPE getCurrentState() {
        return currentState;
    }

    public void setStateToHome() {
        setState(STATE_TELESCOPE.HOME);
    }

    @Override
    public void periodic() {

    }

}