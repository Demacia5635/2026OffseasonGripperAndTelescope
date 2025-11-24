package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2 .command.SubsystemBase;
import frc.Telescop.ConstantsTelescop;
import frc.Telescop.ConstantsTelescop.STATE;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;
//import frc.demacia.utils.Log.LogManager;

public class Telescop extends SubsystemBase {

    private LimitSwitch limitSwitchUp;
    private LimitSwitch limitSwitchDown;
    private TalonMotor motor;

    private double length;

    private STATE currentState = STATE.HOME;

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

    // public void log(){
    //     LogManager.addEntry("trlescope",length,3,"length");
    //     LogManager.addEntry("state", currentState, 3, "State");
    //     LogManager.addEntry("vylosty", motor.getCurrentVelocity(),3,"vylosty");
    //     LogManager.addEntry("pozesan", motor.getCurrentPosition(),3,"pozesan");
    //     LogManager.AddEntry("acsloresan", motor.getCurrentAcceleration(),3,"acsloresan");
    // }

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

    public void Calbresan(){
        motor.setMotion(3);
        if(limitSwitchDown.get() == false){
            motor.setMotion(-0.1);
        }
        length = 0.03;
    }

    public void extendTelescope(double length) {
        this.length = length;
        motor.setMotion(length);
    }
}