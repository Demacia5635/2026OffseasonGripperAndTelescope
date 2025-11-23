package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj2 .command.SubsystemBase;
import frc.Telescop.ConstantsTelescop;  
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;

public class Telescop extends SubsystemBase {

    private LimitSwitch limitSwitchUp;
    private LimitSwitch limitSwitchDown;
    private TalonMotor motor;

    /** Creates a new telescop. */
    public Telescop() {
        limitSwitchUp = new LimitSwitch(ConstantsTelescop.CONFIG_UP);
        limitSwitchDown = new LimitSwitch(ConstantsTelescop.CONFIG_DOWN);
        motor = new TalonMotor(ConstantsTelescop.MOTOR_CONFIG);
    }


    public void rsetEncoder(){
        motor.setEncoderPosition(ConstantsTelescop.reset);
    }

    public void open(){
        if(limitSwitchUp.get() == false){
            motor.setDuty(ConstantsTelescop.lowSpeed);
        }else{
            motor.setDuty(ConstantsTelescop.reset);
        }
    }

    public void startPozesan(){
        if(limitSwitchDown.get() == false){
            motor.setDuty(ConstantsTelescop.lowSpeedMinas);
        }else{
            motor.setDuty(ConstantsTelescop.reset);
            motor.setEncoderPosition(ConstantsTelescop.reset);
        }
    }

    public void Stop(){
        motor.setDuty(ConstantsTelescop.reset);
    }

    public void extendTelescope(double length) {
        motor.setMotion(length);
    }
}
