package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj2 .command.SubsystemBase;

import frc.Telescop.ConstantsTelescop;

import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;

public class Telescop extends SubsystemBase {

    LimitSwitch limitSwitchUp;
    
    LimitSwitch limitSwitchDown;
    private TalonMotor telescopMotor;

    /** Creates a new telescop. */
    public Telescop() {
        limitSwitchUp = new LimitSwitch(ConstantsTelescop.CONFIG_UP);
        limitSwitchDown = new LimitSwitch(ConstantsTelescop.CONFIG_DOWN);
        telescopMotor = new TalonMotor(ConstantsTelescop.MOTOR_CONFIG);
    }


    public void moveToPeckUp(){
        telescopMotor.setMotion(ConstantsTelescop.peckUp);
    }

    public void MoveToL1(){
        telescopMotor.setMotion(ConstantsTelescop.L1);
    }

    public void MoveToL2(){
        telescopMotor.setMotion(ConstantsTelescop.L2);
    }

    public void MoveToL3(){
        telescopMotor.setMotion(ConstantsTelescop.L3);
    }

    public void MoveTOL4(){
        telescopMotor.setMotion(ConstantsTelescop.L4);
    }

    public void MoveToHOME(){
        telescopMotor.setMotion(ConstantsTelescop.HOME);
    }

    public void MoveToTESTING(){
        telescopMotor.setMotion(ConstantsTelescop.TESTING);
    }

    public void MoveToIDLE(){
        telescopMotor.setMotion(ConstantsTelescop.IDLE);
    }
}
