package frc.robot.ChangeAngleArm.subsystems;


import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;
import frc.demacia.utils.Motors.TalonFXMotor;
import frc.demacia.utils.Sensors.AnalogEncoder;
import frc.demacia.utils.Sensors.DigitalEncoder;
import frc.demacia.utils.Sensors.LimitSwitch;
import frc.robot.RobotContainer;
// import frc.demacia.utils.Sensors.AnalogEncoder;
import frc.robot.ChangeAngleArm.ConstantsChangeAngle;
import frc.robot.ChangeAngleArm.ConstantsChangeAngle.TELESCOPE_ANGLE;

public class ChangeAngle extends SubsystemBase {
    // AnalogEncoder changeAngleEncoder;
    DigitalEncoder changeAngleEncoder;
    TalonFXMotor changeAngleMotor;
    TELESCOPE_ANGLE state;
    LimitSwitch limitSwitch;
    final double offset = 0;//-1.5827280362341725;

    @SuppressWarnings("unchecked")
    public ChangeAngle() {
        changeAngleMotor = new TalonFXMotor(ConstantsChangeAngle.CHANGE_ANGLE_CONFIG);
        limitSwitch = new LimitSwitch(ConstantsChangeAngle.SENSOR_CONFIG_ANGLE);
        changeAngleEncoder = new DigitalEncoder(ConstantsChangeAngle.CHANGE_ANGLE_ANALOG_CONFIG);
        // changeAngleEncoder = new AnalogEncoder(ConstantsChangeAngle.CHANGE_ANGLE_ANALOG_CONFIG);
        // calibrate();
        state = TELESCOPE_ANGLE.IDLE;
        changeAngleMotor.setEncoderPosition(changeAngleEncoder.get() - offset);
        addNT();
        LogManager.addEntry("angle motor", () -> getAngleMotor()).withIsSeparated(true).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
        LogManager.addEntry("angle Rev", () -> getAngleSensor()).withIsSeparated(true).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
    }

    public void setAngle(double angle) {
        if (angle < Math.toRadians(-5) || angle > Math.toRadians(90)){ 
            changeAngleMotor.stop();
            return;
            
        }
        if (getSensor()){
            changeAngleMotor.stop();
            return;
        }
        changeAngleMotor.setMotion(angle, ConstantsChangeAngle.KG * Math.cos(getAngleMotor()) + ConstantsChangeAngle.KE * RobotContainer.TelescopSubSystem.getCurrentHeigt() / 0.7);
    }

   

    public double getAngleMotor(){         
        return changeAngleMotor.getCurrentAngle();
    }

    
    public double getAngleSensor(){
        return changeAngleEncoder.get();
    }

    // public double setEncoderPosition(){
    //     changeAngleMotor.
    // }

    public boolean getSensor(){
        return limitSwitch.get();
    }

    public void setPower(double power){
        if (getSensor() && power < 0){
            changeAngleMotor.stop();
            return;
        }
        changeAngleMotor.setDuty(power);
    }

    public TELESCOPE_ANGLE getState(){
        return state;
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
        SmartDashboard.putData("Change Angle State Chooser", stateChooser);
        LogManager.addEntry("Rev encdoer", ()->changeAngleEncoder.get());
    }
}
