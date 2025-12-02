package frc.Telescop.subsystems;



import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2 .command.SubsystemBase;
import frc.Telescop.ConstantsTelescop;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Sensors.LimitSwitch;

public class Telescop extends SubsystemBase {

    //private LimitSwitch limitSwitchUp;
    public static LimitSwitch limitSwitchDown;
    private TalonMotor motor;

    public static boolean calibreated = true;

    public enum STATE{
        L1(0,0),
        L2(0,0),
        L3(0,0),
        L4(0,0),
        HOME(0,0),
        TESTING(-1, -1),
        IDLE(-1, -1),
        INTAKE(0,0),
        CLOSED(-1,0),
        OPEN(-1, -1),
        CALIBRATE(0,0.03);

        

        public double angle;
        public double length;
        STATE(double angle, double length){
            this.angle = angle;
            this.length = length;
        }
    }


    public STATE currentState = STATE.CALIBRATE;

    /** Creates a new telescop. */
    public Telescop() {
        //limitSwitchUp = new LimitSwitch(ConstantsTelescop.CONFIG_UP);
        limitSwitchDown = new LimitSwitch(ConstantsTelescop.CONFIG_DOWN);
        motor = new TalonMotor(ConstantsTelescop.MOTOR_CONFIG);
    }

    private double currentHeigt = motor.getCurrentPosition();

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Length", this::getLength, null);
        builder.addStringProperty("State", ()->currentState.name(), null);
        builder.addDoubleProperty("Test Length", ()->STATE.TESTING.length, (l)->STATE.TESTING.length = l);
    }


    public double getLength() {
        return motor.getCurrentPosition();
    }

    public void setPosition(double l) {
        motor.setEncoderPosition(l);
    }

    public void Stop(){
        motor.setDuty(0);
    }

    public void setPower(double power) {
        motor.setDuty(power);
    }

    public void setLengthPosition(double length) {
        motor.setMotion(length);
    }



    @Override
    public void periodic(){
        LogManager.log("Heigt" + currentHeigt);
    }

}