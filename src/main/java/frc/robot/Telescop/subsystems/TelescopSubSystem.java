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
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.ChangeAngleArm.subsystems.ChangeAngle;
import frc.robot.Telescop.ConstantsTelescop;
import frc.robot.Telescop.ConstantsTelescop.STATE_TELESCOPE;


public class TelescopSubSystem extends SubsystemBase {

    private LimitSwitch limitSwitchTelescope;

    private TalonFXMotor motor;
    public static boolean calibrated = false;

    private static STATE_TELESCOPE currentState = STATE_TELESCOPE.CALIBRATE;

    public static boolean setCalibrated;

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
        builder.addBooleanProperty("Is at limit", ()->isAtBottom(), null);
        builder.addDoubleProperty("Test Length", () -> STATE_TELESCOPE.TESTING.length, (l) -> STATE_TELESCOPE.TESTING.length = l);

    }

    @SuppressWarnings("unchecked")
    public void putData() {
        SendableChooser<STATE_TELESCOPE> stateChooser = new SendableChooser<>();
        stateChooser.addOption("HOME", STATE_TELESCOPE.HOME);
        stateChooser.addOption("INTAKE", STATE_TELESCOPE.INTAKE);
        stateChooser.addOption("CALIBRATE", STATE_TELESCOPE.CALIBRATE);
        stateChooser.addOption("open", STATE_TELESCOPE.OPEN);
        stateChooser.addOption("close", STATE_TELESCOPE.CLOSED);
        stateChooser.addOption("OUT_TAKE", STATE_TELESCOPE.OUT_TAKE);
        stateChooser.addOption("TESTING", STATE_TELESCOPE.TESTING);
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

    public void setMotorLength(double l) {
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
        motor.setPositionVoltage(MathUtil.clamp(wantedLength, MIN_LENGTH, ConstantsTelescop.MAX_LENGTH), Math.sin(RobotContainer.changeAngle.getAngle()));
    }   


    public static boolean isCalibreated() {
        return calibrated;
    }


    public static void setCalibrated(boolean Calibrated) {
        calibrated = Calibrated;
    }

    public boolean isAtBottom() {
        return getSensor();
    }

    public static void setState(STATE_TELESCOPE state) {
        if (isCalibreated()) {
            currentState = state;
        } else {
            LogManager.log("not calibreated");
        }
    }

    public boolean getSensor() {
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