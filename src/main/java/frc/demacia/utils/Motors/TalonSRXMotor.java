package frc.demacia.utils.Motors;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

public class TalonSRXMotor extends TalonSRX implements MotorInterface {
    TalonSRXConfig config;
    String name;

    int slot = 0;

    ControlMode controlMode = ControlMode.DISABLE;

    public TalonSRXMotor(TalonSRXConfig config) {
        super(config.id);
        this.config = config;
        name = config.name;
        configMotor();
        addLog();
        setName(name);
        SmartDashboard.putData(name, this);
        LogManager.log(name + " motor initialized");
    }

    private void configMotor() {
        configFactoryDefault();
        configContinuousCurrentLimit((int) config.maxCurrent);
        configPeakCurrentLimit((int) config.maxCurrent);
        configPeakCurrentDuration(100);
        enableCurrentLimit(true);
        configClosedloopRamp(config.rampUpTime);
        configOpenloopRamp(config.rampUpTime);
        setInverted(config.inverted);
        setNeutralMode(config.brake ? NeutralMode.Brake : NeutralMode.Coast);
        configPeakOutputForward(config.maxVolt / 12.0);
        configPeakOutputReverse(config.minVolt / 12.0);
        configVoltageCompSaturation(config.maxVolt);
        enableVoltageCompensation(true);
    }

    @Override
    public void setName(String name) {
        MotorInterface.super.setName(name);
        this.name = name;
    }

    @SuppressWarnings("unchecked")
    private void addLog() {
      LogManager.addEntry(name + ": position, Velocity, Acceleration, Voltage, Current, CloseLoopError, CloseLoopSP", 
        () -> getCurrentPosition(),
        () -> getCurrentVelocity(),
        () -> getCurrentAcceleration(),
        () -> getCurrentVoltage(),
        () -> getCurrentCurrent(),
        () -> getCurrentClosedLoopError(),
        () -> getCurrentClosedLoopSP()
        ).withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP)
        .withIsMotor().build();
    }

    public void checkElectronics() {
        com.ctre.phoenix.motorcontrol.Faults faults = new com.ctre.phoenix.motorcontrol.Faults();
        getFaults(faults);
        if (faults.hasAnyFault()) {
            LogManager.log(name + " have fault num: " + faults.toString(), AlertType.kError);
        }
    }

    public void changeSlot(int slot){
        if (slot < 0 || slot > 2) {
            LogManager.log("slot is not between 0 and 2", AlertType.kError);
            return;
        }
        this.slot = slot;
    }

    public void setNeutralMode(boolean isBrake){
        setNeutralMode(isBrake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    public void setDuty(double power){
        set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, power);
        if (power == 0){
            controlMode = ControlMode.DISABLE;
        } else {
            controlMode = ControlMode.DUTYCYCLE;
        }
    }

    public void setVoltage(double voltage){
        set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, voltage/12.0);
        controlMode = ControlMode.VOLTAGE;
    }

    public void setVelocity(double velocity, double feedForward){
        LogManager.log("there is no Velocity");
    }

    public void setVelocity(double velocity){
        setVelocity(velocity, 0);
    }

    public void setMotion(double position, double feedForward){
        LogManager.log("there is no motion");
    }

    public void setMotion(double position){
        setMotion(position, 0);
    }

    @Override
    public void setAngle(double angle, double feedForward) {
      setMotion(getCurrentPosition() + MathUtil.angleModulus(angle - getCurrentAngle()), feedForward);
      controlMode = ControlMode.ANGLE;
    }

    @Override
    public void setAngle(double angle) {
      setAngle(angle, 0);
    }

    @Override
    public void setPositionVoltage(double position, double feedForward) {
        LogManager.log("there is no PositionVoltage");
    }

    @Override
    public void setPositionVoltage(double position) {
        setPositionVoltage(position, 0);
    }

    public void setVelocityWithFeedForward(double velocity) {
        setVelocity(velocity, velocityFeedForward(velocity));
    }

    public void setMotionWithFeedForward(double velocity) {
        setMotion(velocity, positionFeedForward(velocity));
    }

    private double velocityFeedForward(double velocity) {
        return velocity * velocity * Math.signum(velocity) * config.kv2;
    }

    private double positionFeedForward(double position) {
        return Math.cos(position * config.posToRad) * config.kSin;
    }

    public int getCurrentControlMode(){
        return controlMode.ordinal();
    }
    
    @Override
    public double getCurrentClosedLoopSP() {
        return getClosedLoopTarget(0) / config.motorRatio;
    }

    @Override
    public double getCurrentClosedLoopError() {
        return getClosedLoopError(0) / config.motorRatio;
    }

    @Override
    public double getCurrentPosition() {
        return getSelectedSensorPosition() / config.motorRatio;
    }

    @Override
    public double getCurrentAngle() {
        if (config.isRadiansMotor) {
            return MathUtil.angleModulus(getCurrentPosition());
        }
        return 0;
    }

    @Override
    public double getCurrentVelocity() {
        return (getSelectedSensorVelocity() * 10.0) / config.motorRatio;
    }

    @Override
    public double getCurrentAcceleration() {
        return 0; // Phoenix 5 SRX doesn't have direct acceleration
    }

    @Override
    public double getCurrentVoltage() {
        return getMotorOutputVoltage();
    }

    @Override
    public double getCurrentCurrent() {
        return getStatorCurrent();
    }

    @Override
    public void setEncoderPosition(double position) {
        setSelectedSensorPosition(position * config.motorRatio);
    }
   
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Talon SRX Motor");
        builder.addDoubleProperty("ControlMode", this::getCurrentControlMode, null);
        builder.addDoubleProperty("Position", this::getCurrentPosition, null);
        builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
        builder.addDoubleProperty("Voltage", this::getCurrentVoltage, null);
        builder.addDoubleProperty("Current", this::getCurrentCurrent, null);
        builder.addDoubleProperty("CloseLoop Error", this::getCurrentClosedLoopError, null);
        if (config.isRadiansMotor) {
            builder.addDoubleProperty("Angle", this::getCurrentAngle, null);
        }
    }

    @Override
    public String getName() {
        return name;
    }

    public double gearRatio() {
        return config.motorRatio;
    }

    public void stop(){
        setDuty(0);
    }
}
