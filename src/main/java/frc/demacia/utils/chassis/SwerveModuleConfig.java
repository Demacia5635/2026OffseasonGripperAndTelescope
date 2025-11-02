package frc.demacia.utils.chassis;

import com.ctre.phoenix6.CANBus;

import frc.demacia.utils.Motors.TalonConfig;
import frc.demacia.utils.Sensors.CancoderConfig;

/**
 * Abstract base class for motor configurations
 * Contains common fields and methods shared between different motor controller types
 */
public abstract class SwerveModuleConfig {

    public String name;             // Name of the motor - used for logging

    public TalonConfig steerConfig;
    public TalonConfig driveConfig;
    public CancoderConfig cancoderConfig;
    public double steerOffset;

        /**
     * Constructor
     * @param id - CAN bus ID
     * @param name - name of motor for logging
     */
    public SwerveModuleConfig(String name, TalonConfig steerConfig, TalonConfig driveConfig, CancoderConfig cancoderConfig) {
        this.name = name;
        this.steerConfig = steerConfig;
        this.driveConfig = driveConfig;
        this.cancoderConfig = cancoderConfig;
    }
    
    public SwerveModuleConfig withSteerOffset(double steerOffset) {
        this.steerOffset = steerOffset;
        return this;
    }
}
