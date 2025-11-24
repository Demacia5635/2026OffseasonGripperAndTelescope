package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2 .command.SubsystemBase;
import frc.Telescop.ConstantsTelescop;
import frc.Telescop.ConstantsTelescop.STATE;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;

public class Telescop extends SubsystemBase {

    private LimitSwitch limitSwitchUp;
    private LimitSwitch limitSwitchDown;
    private TalonMotor motor;

    private double length;

    STATE currentState = STATE.HOME;

    /** Creates a new telescop. */
    public Telescop() {
        limitSwitchUp = new LimitSwitch(ConstantsTelescop.CONFIG_UP);
        limitSwitchDown = new LimitSwitch(ConstantsTelescop.CONFIG_DOWN);
        motor = new TalonMotor(ConstantsTelescop.MOTOR_CONFIG);
    }

    public void SmartDasbord(){
        SmartDashboard.putNumber("length", length);
        SmartDashboard.getNumber("get length", length);
        SmartDashboard.putString("STATE", currentState.name());
        SmartDashboard.getString("get State", currentState.name());
    }

    public void rsetEncoder(){
        motor.setEncoderPosition(0);
    }

    public void Stop(){
        motor.setDuty(0);
    }

    
    public void stopSpeed(){
        motor.setVelocity(0);
    }
    
    public void open(){
        if(limitSwitchUp.get() == false){
            motor.setDuty(0.2);
        }else{
            stopSpeed();
        }
    }

    public void startPozesan(){
        if(limitSwitchDown.get() == false){
            motor.setDuty(-0.2);
        }else{
            stopSpeed();
            rsetEncoder();
        }
    }

    public void extendTelescope(double length) {
        this.length = length;
        motor.setMotion(length);
    }
}
