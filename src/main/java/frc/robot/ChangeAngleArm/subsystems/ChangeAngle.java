package frc.robot.ChangeAngleArm.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonFXMotor;
// import frc.demacia.utils.Sensors.AnalogEncoder;
import frc.robot.ChangeAngleArm.ConstantsChangeAngle;
import frc.robot.ChangeAngleArm.ConstantsChangeAngle.TELESCOPE_ANGLE;

public class ChangeAngle extends SubsystemBase {
    // AnalogEncoder changeAngleEncoder;
    TalonFXMotor changeAngleMotor;
    TELESCOPE_ANGLE state;
    double offset = 0;

    public ChangeAngle() {
        this.changeAngleMotor = new TalonFXMotor(ConstantsChangeAngle.CHANGE_ANGLE_CONFIG);
        // this.changeAngleEncoder = new AnalogEncoder(ConstantsChangeAngle.CHANGE_ANGLE_ANALOG_CONFIG);
        // calibrate();
        state = TELESCOPE_ANGLE.IDLE;
        addNT();
    }

    public void setAngle(double angle) {
        changeAngleMotor.setMotion(angle + offset, ConstantsChangeAngle.KG * Math.cos(getAngle()));
    }

    public double getAngle(){
        return changeAngleMotor.getCurrentAngle();
    }

    public void setPower(double power){
        changeAngleMotor.setDuty(power);
    }

    public TELESCOPE_ANGLE getState(){
        return state;
    }

    // public void calibrate(){
    //     offset = changeAngleEncoder.get() - getAngle();
    // }

    public void addNT(){
        SendableChooser<TELESCOPE_ANGLE> stateChooser = new SendableChooser<>();
        stateChooser.addOption("IDLE2", TELESCOPE_ANGLE.IDLE);
        for (TELESCOPE_ANGLE state : TELESCOPE_ANGLE.values()) {
            stateChooser.addOption(state.name(), state);
        }
        stateChooser.onChange(state -> this.state = state);
        SmartDashboard.putData("Change Angle State Chooser", stateChooser);
    }
}
