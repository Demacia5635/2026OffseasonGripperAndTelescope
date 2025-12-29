package frc.demacia.utils.Sensors;

import edu.wpi.first.wpilibj.Compressor;

public class Pneumatics extends Compressor {
 PneumaticsConfig config;
 String name;
	
public Pneumatics(PneumaticsConfig config) {
        super(config.module, config.moduleType);
    this.config = config;
    this.name= config.name;
    }
    public String getName() {
        return config.name;   
    }
    public boolean getCompressorState() {
        return super.isEnabled();
    }
    public boolean getPressureSwitch() {
        return super.getPressureSwitchValue();
    }
    public double getCurrent() {
        return super.getCurrent();
    }
    public double getAnalogVoltage() {
        return super.getAnalogVoltage();
    }
    public double getAnalogPressure() {
        return super.getPressure();
    }
    public void disableCompressor() {
        super.disable();
    }
    public void enableCompressorDigital() {
        super.enableDigital();
    }
    public void enableCompressorAnalog(double minPressure, double maxPressure) {
        super.enableAnalog(minPressure, maxPressure);
    }
    public void enableHybrid(double minPressure, double maxPressure) {
        super.enableHybrid(minPressure, maxPressure);
    }

}



