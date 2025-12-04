package frc.robot.Telescop.subsystems;

import static frc.robot.Telescop.ConstantsTelescop.*;

import java.util.function.BiConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogManager2;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;
import frc.robot.Telescop.ConstantsTelescop;

public class Telescop extends SubsystemBase {

    // private LimitSwitch limitSwitchUp;
    private LimitSwitch limitSwitchTelescope;

    private TalonMotor motor;

    private boolean calibreated;

    private STATE currentState = STATE.CALIBRATE;
    private double currentHeigt;

    /** Creates a new telescop. */
    public Telescop() {
        limitSwitchTelescope = new LimitSwitch(ConstantsTelescop.CONFIG_DOWN);
        motor = new TalonMotor(ConstantsTelescop.MOTOR_CONFIG);
        calibreated = false;
        currentHeigt = motor.getCurrentPosition();
        putData();
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Length", this::getCurrentLength, null);
        builder.addStringProperty("State", () -> currentState.name(), null);
        builder.addDoubleProperty("Test Length", () -> STATE.TESTING.length, (l) -> STATE.TESTING.length = l);
    }

    public void putData() {
        SendableChooser<STATE> stateChooser = new SendableChooser<>();
        stateChooser.addOption("L1", STATE.L1);
        stateChooser.addOption("L2", STATE.L2);
        stateChooser.addOption("L3", STATE.L3);
        stateChooser.addOption("L4", STATE.L4);
        stateChooser.addOption("HOME", STATE.HOME);
        stateChooser.addOption("INTAKE", STATE.INTAKE);
        stateChooser.addOption("CALIBRATE", STATE.CALIBRATE);
        stateChooser.onChange(this::setState);
        SmartDashboard.putData("Telescop State", stateChooser);

        LogManager2.addEntry("Telescope", this::getCurrentLength, this::isAtBottom, this::isCalibreated);
    }

    public double getCurrentLength() {
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

    public void setLengthPosition(double wantedLength) {
        if (!isCalibreated()) {
            LogManager.log("Telescop not calibrated");
            stop();
            return;
        }
        if (isAtBottom()) {
            LogManager.log("Telescop at limit switch");
            stop();
            return;
        }
        // if (getCurrentLength() < MIN_LENGTH || getCurrentLength() > ConstantsTelescop.MAX_LENGTH) {
        //     LogManager.log("Telescop out of bounds");
        //     stop();
        //     return;
        // }
        motor.setMotion(MathUtil.clamp(wantedLength, MIN_LENGTH, ConstantsTelescop.MAX_LENGTH));
    }

    public boolean isCalibreated() {
        return calibreated;
    }

    public void setCalibrated() {
        this.calibreated = true;
    }

    public boolean isAtBottom() {
        return limitSwitchTelescope.get();
    }

    public void setState(STATE state) {
        this.currentState = state;
    }

    public STATE getCurrentState() {
        return this.currentState;
    }

    @Override
    public void periodic() {
    }

}