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

    public void startPozesan(){
        if (limitSwitchDown.get() == false){
            while(limitSwitchDown.get() != true){
                motor.setDuty(-0.2);
            }
            motor.setEncoderPosition(0);
        }else{
            motor.setEncoderPosition(0);
        }
    }

    public void open(){
        while(limitSwitchUp.get() == false){
            motor.setDuty(0.5);
        }
    }

    public void close(){
        while(limitSwitchDown.get() == false){
            motor.setDuty(-0.5);
        }
    }

    public void Stop(){
        motor.setDuty(0);
    }

    public void extendTelescope(double value) {
        motor.setMotion(value);
    }
}
