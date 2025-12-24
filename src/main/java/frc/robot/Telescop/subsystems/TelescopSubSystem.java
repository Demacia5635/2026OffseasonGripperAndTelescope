package frc.robot.Telescop.subsystems;

import static frc.robot.Telescop.ConstantsTelescop.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
import frc.demacia.utils.Motors.TalonFXMotor;
import frc.demacia.utils.Sensors.LimitSwitch;
import frc.robot.Telescop.ConstantsTelescop;
import frc.robot.Telescop.ConstantsTelescop.STATE;


public class TelescopSubSystem extends SubsystemBase {

    private LimitSwitch limitSwitchTelescope;

    private TalonFXMotor motor;
    public static boolean calibrated = false;

    private static STATE currentState = STATE.CALIBRATE;

    /** Creates a new telescop. */
    public TelescopSubSystem() {
        limitSwitchTelescope = new LimitSwitch(ConstantsTelescop.SENSOR_CONFIG);
        motor = new TalonFXMotor(ConstantsTelescop.MOTOR_CONFIG);
        putData();
        SmartDashboard.putData("Telescop", this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addBooleanProperty("Sensor", this::getSensor, null);
        builder.addDoubleProperty("Length", this::getCurrentHeigt, null);
        builder.addStringProperty("State", () -> currentState.name(), null);
        builder.addDoubleProperty("Test Length", () -> STATE.TESTING.length, (l) -> STATE.TESTING.length = l);

    }

    @SuppressWarnings("unchecked")
    public void putData() {
        SendableChooser<STATE> stateChooser = new SendableChooser<>();
        stateChooser.addOption("L1", STATE.L1);
        stateChooser.addOption("L2", STATE.L2);
        stateChooser.addOption("L3", STATE.L3);
        stateChooser.addOption("L4", STATE.L4);
        stateChooser.addOption("HOME", STATE.HOME);
        stateChooser.addOption("INTAKE", STATE.INTAKE);
        stateChooser.addOption("CALIBRATE", STATE.CALIBRATE);
        stateChooser.onChange(STATE -> setState(STATE));
        SmartDashboard.putData("Telescop State", stateChooser);
        LogManager.addEntry("Telescope", () -> (new double[] { getCurrentHeigt() }))
                .withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();

        LogManager.addEntry("Telescope2", () -> (new boolean[] { isAtBottom(), isCalibreated() }))
                .withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();

        motor.showConfigMotionVelocitiesCommand();
        motor.showConfigPIDFSlotCommand(0);
    }
    
    public double getCurrentHeigt() {
        return motor.getCurrentPosition();
    }

    public void setMotorPosition(double l) {
        motor.setEncoderPosition(l);
    }

    public void stop() {
        motor.setDuty(0);
    }

    public void setPower(double power) {
        motor.setDuty(power);
    }

    public double getVelocity() {
        return motor.getCurrentVelocity();
    }

    public void testPosition(double pos) {
        motor.setPositionVoltage(pos);
    }

    public void setLengthHeigt(double wantedLength) {
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
        motor.setPositionVoltageWithFeedForward(MathUtil.clamp(wantedLength, MIN_LENGTH, ConstantsTelescop.MAX_LENGTH));
    }

    public static boolean isCalibreated() {
        return calibrated;
    }

    public void setCalibrated() {
        calibrated = true;
    }

    public boolean isAtBottom() {
        if (limitSwitchTelescope.get()){
            return true;
        }else{
            return false;
        }
    }

    public static void setState(STATE state) {
        if (isCalibreated()) {
            currentState = state;
        } else {
            LogManager.log("not calibreated");
        }
    }

    public boolean getSensor() {
        return limitSwitchTelescope.get();
    }

    public STATE getCurrentState() {
        return currentState;
    }

    public void setStateToHome() {
        setState(STATE.HOME);
    }

    @Override
    public void periodic() {

    }

}