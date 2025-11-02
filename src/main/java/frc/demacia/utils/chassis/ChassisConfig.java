package frc.demacia.utils.chassis;

import edu.wpi.first.math.geometry.Translation2d;
import frc.demacia.utils.Sensors.PigeonConfig;

public class ChassisConfig {
    public SwerveModuleConfig frontLeftModuleConfig;
    public SwerveModuleConfig frontRightModuleConfig;
    public SwerveModuleConfig backLeftModuleConfig;
    public SwerveModuleConfig backRightModuleConfig;

    public PigeonConfig pigeonConfig;

    public Translation2d frontLeftPosition;
    public Translation2d frontRightPosition;
    public Translation2d backLeftPosition;
    public Translation2d backRightPosition;

    public double CYCLE_DT = 0.02;
    public double MAX_LINEAR_ACCEL = 10;
    public double MAX_OMEGA_VELOCITY = Math.toRadians(540);
    public double MAX_RADIAL_ACCEL = 6;
    public double MAX_RADIUS = 0.4;
    public double MIN_OMEGA_DIFF = Math.toRadians(20);
    public double MAX_DELTA_VELOCITY = MAX_LINEAR_ACCEL * CYCLE_DT;
    public double MAX_VELOCITY_TO_IGNORE_RADIUS = MAX_RADIUS * MAX_OMEGA_VELOCITY;
    public double MIN_VELOCITY = 1.5;
}
