package frc.demacia.utils.Sensors;

import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Base interface for all sensors.
 * 
 * <p>Provides common methods that all sensor types must implement for
 * identification and health monitoring.</p>
 */
public  interface SensorInterface{
    /**
     * Gets the sensor's configured name.
     * 
     * @return Sensor name as specified in configuration
     */
    String getName();
    
    /**
     * Checks sensor health and logs any faults.
     * 
     * <p>Should be called periodically (e.g., in subsystem periodic() method).
     * Logs warnings or errors if sensor is disconnected, reporting faults, etc.</p>
     */
    public void checkElectronics();

    void initSendable(SendableBuilder builder);
}