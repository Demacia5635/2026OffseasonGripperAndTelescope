package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj2 .command.SubsystemBase;

import frc.Telescop.ConstantsTelescop;  

import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;

public class Telescop extends SubsystemBase {

    private LimitSwitch limitSwitchUp;
    private LimitSwitch limitSwitchDown;
    private TalonMotor Motor;

    /** Creates a new telescop. */
    public Telescop() {
        limitSwitchUp = new LimitSwitch(ConstantsTelescop.CONFIG_UP);
        limitSwitchDown = new LimitSwitch(ConstantsTelescop.CONFIG_DOWN);
        Motor = new TalonMotor(ConstantsTelescop.MOTOR_CONFIG);
    }


    public void Stop(){
        Motor.setDyte(0);
    }

    public void extendTelescope(double value) {
        Motor.setMotion(value);
    }
}
