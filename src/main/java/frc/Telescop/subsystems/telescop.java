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

    public void extendTelescope(double value) {
        telescopMotor.setMotion(value);
    }
}
