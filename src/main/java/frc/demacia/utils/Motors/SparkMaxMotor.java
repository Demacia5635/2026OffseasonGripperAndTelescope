package frc.demacia.utils.Motors;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

public class SparkMaxMotor extends SparkMax implements MotorInterface {

  private frc.demacia.utils.Motors.SparkMaxConfig config;
  private String name;
  private SparkMaxConfig cfg;
  private ClosedLoopSlot closedLoopSlot = ClosedLoopSlot.kSlot0;
  private ControlType controlType = ControlType.kDutyCycle;

  private ControlMode controlMode = ControlMode.DISABLE;
  private double lastVelocity;
  private double lastAcceleration;
  private double setPoint = 0;
  private double lastTime = 0;

  public SparkMaxMotor(frc.demacia.utils.Motors.SparkMaxConfig config) {
    super(config.id, SparkLowLevel.MotorType.kBrushless);
    this.config = config;
    name = config.name;
    configMotor();
    addLog();
    setName(name);
    SmartDashboard.putData(name, this);
    LogManager.log(name + " motor initialized");
  }

  private void configMotor() {
    cfg = new SparkMaxConfig();
    cfg.smartCurrentLimit((int) config.maxCurrent);
    cfg.openLoopRampRate(config.rampUpTime);
    cfg.closedLoopRampRate(config.rampUpTime);
    cfg.inverted(config.inverted);
    cfg.idleMode(config.brake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    cfg.voltageCompensation(config.maxVolt);
    cfg.encoder.positionConversionFactor(1 / config.motorRatio);
    cfg.encoder.velocityConversionFactor(1 / config.motorRatio);
    updatePID(false);
    if (config.maxVelocity != 0) {
      cfg.closedLoop.maxMotion.maxVelocity(config.maxVelocity).maxAcceleration(config.maxAcceleration);
    }
    this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  private void updatePID(boolean apply) {
    cfg.closedLoop.pidf(config.pid[0].kP(), config.pid[0].kI(), config.pid[0].kD(), config.pid[0].kV(),
        ClosedLoopSlot.kSlot0);
    cfg.closedLoop.pidf(config.pid[1].kP(), config.pid[1].kI(), config.pid[1].kD(), config.pid[1].kV(),
        ClosedLoopSlot.kSlot1);
    cfg.closedLoop.pidf(config.pid[2].kP(), config.pid[2].kI(), config.pid[2].kD(), config.pid[2].kV(),
        ClosedLoopSlot.kSlot2);
    cfg.closedLoop.pidf(config.pid[3].kP(), config.pid[3].kI(), config.pid[3].kD(), config.pid[3].kV(),
        ClosedLoopSlot.kSlot3);
    if (apply) {
      this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public void setName(String name) {
      MotorInterface.super.setName(name);
      this.name = name;
  }

  @SuppressWarnings("unchecked")
  private void addLog() {
    LogManager.addEntry(name + ": Position, Velocity, Acceleration, Voltage, Current, CloseLoopError, CloseLoopSP", 
        () -> getCurrentPosition(),
        () -> getCurrentVelocity(),
        () -> getCurrentAcceleration(),
        () -> getCurrentVoltage(),
        () -> getCurrentCurrent(),
        () -> getCurrentClosedLoopError(),
        () -> getCurrentClosedLoopSP(),
        () -> getCurrentControlMode()
      ).withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP)
      .withIsMotor()
      .build();
  }

  public void checkElectronics() {
    Faults faults = getFaults();
    boolean hasFault = faults.other || faults.motorType || faults.sensor || 
      faults.can || faults.temperature;

    if (hasFault) {
        LogManager.log(name + " Fault Detected: " + faults.toString(), AlertType.kError);
    }
  }

  /**
   * change the slot of the pid and feed forward.
   * will not work if the slot is null
   * 
   * @param slot the wanted slot between 0 and 2
   */
  public void changeSlot(int slot) {
    if (slot < 0 || slot > 3) {
      LogManager.log("slot is not between 0 and 2", AlertType.kError);
      return;
    }
    this.closedLoopSlot = slot == 0 ? ClosedLoopSlot.kSlot0 : slot == 1 ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot2;
  }

  /*
   * set motor to brake or coast
   */
  public void setNeutralMode(boolean isBrake) {
    cfg.idleMode(isBrake ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast);
    configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * set power from 1 to -1 (v/12) no PID/FF
   * 
   * @param power the wanted power between -1 to 1
   */
  public void setDuty(double power) {
    super.set(power);
    controlType = ControlType.kDutyCycle;
    if (power == 0){
      controlMode = ControlMode.DISABLE;
  } else {
      controlMode = ControlMode.DUTYCYCLE;
  }
  }

  public void setVoltage(double voltage) {
    super.setVoltage(voltage);
    controlType = ControlType.kVoltage;
    controlMode = ControlMode.VOLTAGE;
  }

  /**
   * set volocity to motor with PID and FF
   * 
   * @param velocity    the wanted velocity in meter per second or radians per
   *                    seconds depending on the config
   * @param feedForward wanted feed forward to add to the ks kv ka and kg,
   *                    defaults to 0
   */
  public void setVelocity(double velocity, double feedForward) {
    if (config.maxVelocity == 0) {
      LogManager.log(name + ": maxVelocity not configured", AlertType.kError);
      return;
    }
    getClosedLoopController().setReference(velocity, ControlType.kMAXMotionVelocityControl, closedLoopSlot, feedForward);
    controlType = ControlType.kMAXMotionVelocityControl;
    controlMode = ControlMode.VELOCITY;
    setPoint = velocity;
  }

  public void setVelocity(double velocity) {
    setVelocity(velocity, config.pid[closedLoopSlot.value].kS()*Math.signum(velocity));
  }

  public void setPositionVoltage(double position, double feedForward) {
    getClosedLoopController().setReference(position, ControlType.kPosition, closedLoopSlot, feedForward);
    controlType = ControlType.kPosition;
    controlMode = ControlMode.POSITION_VOLTAGE;
    setPoint = position;
  }

  public void setPositionVoltage(double position) {
    setPositionVoltage(position, 0);
  }

  public void setVelocityWithFeedForward(double velocity) {
    setVelocity(velocity, velocityFeedForward(velocity));
  }

  public void setMotionWithFeedForward(double velocity) {
    setVelocity(velocity, positionFeedForward(velocity));
  }

  @Override
  public void setMotion(double position, double feedForward) {
    if (config.maxVelocity == 0) {
      LogManager.log(name + ": maxVelocity not configured", AlertType.kError);
      return;
    }
    getClosedLoopController().setReference(position, ControlType.kMAXMotionPositionControl, closedLoopSlot, feedForward);
    controlType = ControlType.kMAXMotionPositionControl;
    controlMode = ControlMode.POSITION_VOLTAGE;
    setPoint = position;
  }

  @Override
  public void setMotion(double position) {
    setMotion(position, config.pid[closedLoopSlot.value].kS());
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

  private double velocityFeedForward(double velocity) {
    return velocity * velocity * Math.signum(velocity) * config.kv2;
  }

  private double positionFeedForward(double position) {
    return Math.cos(position * config.posToRad) * config.kSin;
  }

  @Override
  public int getCurrentControlMode() {
    return controlMode.ordinal();
  }

  @Override
  public double getCurrentClosedLoopError() {
    switch (controlType) {
      case kPosition, kMAXMotionPositionControl:
        return setPoint - getCurrentPosition();
      case kVelocity, kMAXMotionVelocityControl:
        return setPoint - getCurrentVelocity();
      default:
        return 0;
    }
  }

  @Override
  public double getCurrentClosedLoopSP() {
    return setPoint;
  }

  public double getCurrentPosition() {
    return getEncoder().getPosition();
  }

  public double getCurrentAngle() {
    if (config.isRadiansMotor) {
      return MathUtil.angleModulus(getCurrentPosition());
    }
    return 0;
  }

  public double getCurrentVelocity() {
    double velocity = getEncoder().getVelocity();
    double time = Timer.getFPGATimestamp();
    double dt = time - lastTime;
    if (dt > 0) {
        lastAcceleration = (velocity - lastVelocity) / dt;
        lastTime = time;
        lastVelocity = velocity;
    }
    return velocity;
  }

  public double getCurrentAcceleration() {
    return lastAcceleration;
  }

  public double getCurrentVoltage() {
    return getAppliedOutput() * 12;
  }
  public double getCurrentCurrent() {
    return getOutputCurrent();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Spark Motor");
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
	  
  /**
   * creates a widget in elastic of the pid and ff for hot reload
   * @param slot the slot of the close loop perams (from 0 to 2)
   */
  public void configPidFf(int slot) {

    Command configPidFf = new InstantCommand(()-> {
      cfg = new SparkMaxConfig();
      closedLoopSlot = slot == 0 ? ClosedLoopSlot.kSlot0 : slot == 1 ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot2;
      cfg.closedLoop.pidf(config.pid[slot].kP(), config.pid[slot].kI(), config.pid[slot].kD(), config.pid[slot].kV(),
        closedLoopSlot);
      this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }).ignoringDisable(true);

    SmartDashboard.putData(name + "/PID+FF config", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PID+FF Config");
          builder.addDoubleProperty("KP", ()-> config.pid[0].kP(), (double newValue) -> config.pid[0].setKP(newValue));
          builder.addDoubleProperty("KI", ()-> config.pid[0].kI(), (double newValue) -> config.pid[0].setKI(newValue));
          builder.addDoubleProperty("KD", ()-> config.pid[0].kD(), (double newValue) -> config.pid[0].setKD(newValue));
          builder.addDoubleProperty("KS", ()-> config.pid[0].kS(), (double newValue) -> config.pid[0].setKS(newValue));
          builder.addDoubleProperty("KV", ()-> config.pid[0].kV(), (double newValue) -> config.pid[0].setKV(newValue));
          builder.addDoubleProperty("KA", ()-> config.pid[0].kA(), (double newValue) -> config.pid[0].setKA(newValue));
          builder.addDoubleProperty("KG", ()-> config.pid[0].kG(), (double newValue) -> config.pid[0].setKG(newValue));
        
        builder.addBooleanProperty("Update", ()-> configPidFf.isScheduled(), 
          value -> {
            if (value) {
              if (!configPidFf.isScheduled()) {
                configPidFf.schedule();
              }
            } else {
              if (configPidFf.isScheduled()) {
                configPidFf.cancel();
              }
            }
          }
        );
      }
    });
  }

  /**
   * creates a widget in elastic to configure motion magic in hot reload
   */
  public void configMotionMagic() {
    Command configMotionMagic = new InstantCommand(()-> {
      cfg = new SparkMaxConfig();
      
      cfg.closedLoop.maxMotion.maxVelocity(config.maxVelocity).maxAcceleration(config.maxAcceleration);
      
      this.configure(cfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }).ignoringDisable(true);
    
    SmartDashboard.putData(name + "/Motion Magic Config", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Motion Magic Config");
        
        builder.addDoubleProperty("Vel", ()-> config.maxVelocity, value-> config.maxVelocity = value);
        builder.addDoubleProperty("Acc", ()-> config.maxAcceleration, value-> config.maxAcceleration = value);
        
        builder.addBooleanProperty("Update", ()-> configMotionMagic.isScheduled(), 
        value -> {
          if (value) {
            if (!configMotionMagic.isScheduled()) {
              configMotionMagic.schedule();
            }
          } else {
            if (configMotionMagic.isScheduled()) {
              configMotionMagic.cancel();
            }
          }
        }
        );
      }
    });
  }

  public double gearRatio() {
    return config.motorRatio;
  }

  public String getName() {
    return name;
  }

  @Override
  public void setEncoderPosition(double position) {
    getEncoder().setPosition(position);
  }

  public void stop(){
      stopMotor();
      controlMode = ControlMode.DISABLE;
  }
}