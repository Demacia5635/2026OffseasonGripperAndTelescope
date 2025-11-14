package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Telescop.ConstantsTelescop;
import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Motors.TalonMotor;
import frc.demacia.utils.Motors.BaseMotorConfig.Canbus;
import frc.Telescop.Constants;

public class telescop extends SubsystemBase {

  ConstantsTelescop constants = new ConstantsTelescop();
  Constants mainConstants = new Constants();
  telescopEncoder encoder = new telescopEncoder();

    private TalonMotor telescopMotor;

    /** Creates a new telescop. */
    public telescop() {
        subsystemsTelescope();
    }

    private void subsystemsTelescope() {
        TalonConfig config = new TalonConfig(0, Canbus.Rio, "telescopMotor")
            .withBrake(false)
            .withVolts(12)
            .withDegreesMotor(14)
            .withPID(0, 0, 0, 0, 0, 0, 0)
            .withMotionParam(constants.getMaxVelocity(), constants.getMaxAcceleration(), constants.getMaxJerk());

        telescopMotor = new TalonMotor(config);
    }


    public void updateState() {
        switch (mainConstants.getState()) {
            case "open":
                telescopMotor.setMotion(constants.open);
            case "close":
                telescopMotor.setMotion(constants.close);
                encoder.resetEncoder();
            case "L1":
                telescopMotor.setMotion(constants.getL1());
            case "L2":
                telescopMotor.setMotion(constants.getL2());
            case "L3":
                telescopMotor.setMotion(constants.getL3());
            case "L4":
                telescopMotor.setMotion(constants.getL4());
        }
    }

    public double currentVelocity() {
        return telescopMotor.getCurrentVelocity();
    }

    @Override
    public void periodic() {
        updateState();
    }
}
