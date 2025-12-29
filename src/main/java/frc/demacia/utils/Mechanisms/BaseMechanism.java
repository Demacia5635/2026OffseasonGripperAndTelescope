package frc.demacia.utils.Mechanisms;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Sensors.SensorInterface;

public class BaseMechanism extends SubsystemBase{
    protected String name;
    protected HashMap<String, MotorInterface> motors;
    protected HashMap<String, SensorInterface> sensors;

    public BaseMechanism(String name, MotorInterface[] motors, SensorInterface[] sensors) {
        this.name = name;
        setName(name);
        this.motors = new HashMap<>();
        for (MotorInterface motor : motors) {
            this.motors.put(motor.getName(), motor);
        }
        this.sensors = new HashMap<>();
        for (SensorInterface sensor : sensors) {
            this.sensors.put(sensor.getName(), sensor);
        }
        for (String motorName : this.motors.keySet()) {
            SmartDashboard.putData(getName() + "/" + motorName + "/set brake", 
                new InstantCommand(() -> setNeutralMode(motorName, true)).ignoringDisable(true));
        }
        for (String motorName : this.motors.keySet()) {
            SmartDashboard.putData(getName() + "/" + motorName + "/set coast", 
                new InstantCommand(() -> setNeutralMode(motorName, false)).ignoringDisable(true));
        }
        SmartDashboard.putData(getName() + "/set coast all", 
                new InstantCommand(() -> setNeutralModeAll(false)).ignoringDisable(true));
        SmartDashboard.putData(getName() + "/set brake all", 
                new InstantCommand(() -> setNeutralModeAll(true)).ignoringDisable(true));
        SmartDashboard.putData(name, this);
    }

    public String getName(){
        return name;
    }

    /**
     * Stops all motors immediately.
     */
    public void stopAll(){
        if (motors == null) return;
        for (MotorInterface motor : motors.values()){
            motor.stop();
        }
    }

    /**
     * Stops a specific motor by index.
     * 
     * @param motorIndex Index of motor to stop
     */
    public void stop(String motorName){
        if (isValidMotorIndex(motorName)){
            motors.get(motorName).setDuty(0);
        }
    }

    /**
     * Sets power for all motors.
     * 
     * @param power Duty cycle (-1.0 to 1.0)
     */
    public void setPowerAll(double power) {
        if (motors == null) return;
        for (MotorInterface motor : motors.values()){
            motor.setDuty(power);
        }
    }

    /**
     * Sets power for a specific motor.
     * 
     * @param motorIndex Index of motor
     * @param power Duty cycle (-1.0 to 1.0)
     */
    public void setPower(String motorName, double power){
        if (isValidMotorIndex(motorName)){
            motors.get(motorName).setDuty(power);
        }
    }

    /**
     * Sets neutral mode for all motors.
     * 
     * @param isBrake true for brake, false for coast
     */
    public void setNeutralModeAll(boolean isBrake) {
        if (motors == null) return;
        for (MotorInterface motor : motors.values()) {
            if (motor != null) motor.setNeutralMode(isBrake);
        }
    }

    public void setNeutralMode(String motorName, boolean isBrake){
        if (isValidMotorIndex(motorName)){
            motors.get(motorName).setNeutralMode(isBrake);
        }
    }

    /**
     * Checks electronics for all motors and sensors.
     * 
     * <p>Logs any faults to console and telemetry.</p>
     */
    public void checkElectronicsAll() {
        if (motors == null) return;
        for (MotorInterface motor : motors.values()) {
            if (motor != null) motor.checkElectronics();
        }
        if (sensors == null) return;
        for (SensorInterface sensor : sensors.values()) {
            if (sensor != null) sensor.checkElectronics();
        }
    }

    public void checkElectronicsMotor(String motorName){
        if (isValidMotorIndex(motorName)){
            motors.get(motorName).checkElectronics();
        }
    }

    public void checkElectronicsSensor(String sensorName){
        if (isValidSensorIndex(sensorName)){
            sensors.get(sensorName).checkElectronics();
        }
    }

    /**
     * Gets a motor by index.
     * 
     * @param index Motor index
     * @return The motor interface
     */
    public MotorInterface getMotor(String motorName) {
        if (!isValidMotorIndex(motorName)){
            LogManager.log("Invalid motor: " + motorName);
            return null;
        }
        return motors.get(motorName);
    }

    /**
     * Gets a sensor by index.
     * 
     * @param index Sensor index
     * @return The sensor interface
     */
    public SensorInterface getSensor(String sensorName) {
        if (!isValidSensorIndex(sensorName)){
            LogManager.log("Invalid sensor: " + sensorName);
            return null;
        }
        return sensors.get(sensorName);
    }

    protected boolean isValidMotorIndex(String motorName) {
        return motors.containsKey(motorName);
    }

    protected boolean isValidSensorIndex(String sensorName) {
        return sensors.containsKey(sensorName);
    }
}
