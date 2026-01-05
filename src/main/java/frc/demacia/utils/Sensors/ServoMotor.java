package frc.demacia.utils.Sensors;

import edu.wpi.first.wpilibj.Servo;

public class ServoMotor extends Servo {

     ServoMotorConfig config;
    String name;
    public ServoMotor (ServoMotorConfig config) {
        super(config.id);
        this.config = config;
        name=config.name;
    }
        @Override
        public double getPosition() {
            return super.getPosition();
        }
        public void setPosition(double position) {
            super.setPosition(position);
        }

        public void setAngle(double angle) {
            super.setAngle(angle);
        }
        public double getAngle() {
            return super.getAngle();
        }
    }
    
