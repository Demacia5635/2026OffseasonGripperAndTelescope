package frc.robot.ChangeAngleArm.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;
import frc.robot.ChangeAngleArm.ConstantsChangeAngle;

public class ChangeAngle extends SubsystemBase {
    TalonMotor changeAngleMotor;
    LimitSwitch limitSwitch;

    public ChangeAngle() {
        this.changeAngleMotor = new TalonMotor(ConstantsChangeAngle.CHANGE_ANGLE_CONFIG);
        this.limitSwitch = new LimitSwitch(ConstantsChangeAngle.CHANGE_ANGLE_LIMIT_CONFIG);
    }

    public void setTargetAngle(double angle) {
        changeAngleMotor.setMotion(angle);
    }

    public double getAngle(){
        return changeAngleMotor.getCurrentAngle();
    }

    public void setPower(double power){
        changeAngleMotor.setDuty(power);
    }
}
