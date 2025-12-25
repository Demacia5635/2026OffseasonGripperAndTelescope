package frc.robot.ChangeAngleArm.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
// import frc.demacia.utils.Sensors.AnalogEncoder;
import frc.robot.ChangeAngleArm.ConstantsChangeAngle;
import frc.robot.ChangeAngleArm.ConstantsChangeAngle.TELESCOPE_ANGLE;

public class ChangeAngle extends SubsystemBase {
    // AnalogEncoder changeAngleEncoder;
    TalonMotor changeAngleMotor;
    DutyCycleEncoder changeAngleEncoder;
    TELESCOPE_ANGLE state;

    public ChangeAngle() {
        setName("Change Angle Subsystem");
        this.changeAngleMotor = new TalonMotor(ConstantsChangeAngle.CHANGE_ANGLE_CONFIG);
        this.changeAngleEncoder = new DutyCycleEncoder(ConstantsChangeAngle.ANALOG_ENCODER_CHANNEL);
        // calibrate();
        state = TELESCOPE_ANGLE.IDLE;
        addNT();
    }

    public void setAngle(double angle) {
        changeAngleMotor.setMotion(angle, ConstantsChangeAngle.KG * Math.cos(getAngle()));
    }

    public double getAngle(){
        
        return -(changeAngleEncoder.get()*2*Math.PI) + ConstantsChangeAngle.ANALOG_ENCODER_OFFSET;
    }

    public void setPower(double power){
        changeAngleMotor.setDuty(power);
    }

    public TELESCOPE_ANGLE getState(){
        return state;
    }

    public void stop(){
        changeAngleMotor.stopMotor();
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
        SmartDashboard.putData(getName()+"/Change Angle State Chooser", stateChooser);
        SmartDashboard.putData(this);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Angle", this::getAngle, null);
    }

}
