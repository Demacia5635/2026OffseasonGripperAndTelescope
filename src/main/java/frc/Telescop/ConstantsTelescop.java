package frc.Telescop;

import edu.wpi.first.wpilibj.Timer;
import frc.Telescop.subsystems.telescop;

public class ConstantsTelescop {

    private telescop telescop = new telescop();

    public static final double RPM = 6000;
    public static final double maxVelocity = (Math.PI * 2) * RPM;

    private double lastVelocity = 0;
    private double lastTime = Timer.getFPGATimestamp();
    private double lastAcceleration = 0;

    private double maxAcceleration = 0;
    private double jerk = 0;
    private double maxJerk = 0;

    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        double currentVelocity = telescop.currentVelocity();

        double deltaV = currentVelocity - lastVelocity;
        double deltaT = currentTime - lastTime;

        
            double acceleration = deltaV / deltaT;
            jerk = (acceleration - lastAcceleration) / deltaT;

            maxJerk = maxAcceleration / deltaV;

            if(acceleration > maxAcceleration){
                maxAcceleration = acceleration;
            }


            lastVelocity = currentVelocity;
            lastTime = currentTime;
            lastAcceleration = acceleration;
 }       
    

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxVelocity(){
        return maxVelocity;
    }

    public double getMaxJerk(){
        return maxJerk;
    }
}
