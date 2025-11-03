package frc.robot.subsystems;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.robot.Constants;

public class trlscopMotor {

    TalonMotor trlscopMotor;
    Constants Constants = new Constants();
    public void motor(){
        TalonConfig TalonConfig= new TalonConfig(
            10,
            Canbus.Rio,
            "trlscopMotor"
        );

        trlscopMotor = new TalonMotor(TalonConfig);
    }
}
