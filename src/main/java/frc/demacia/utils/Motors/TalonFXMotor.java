package frc.demacia.utils.Motors;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.demacia.utils.Data;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

public class TalonFXMotor extends TalonFX implements MotorInterface {

    TalonFXConfig config;
    String name;
    TalonFXConfiguration cfg;

    int slot = 0;

    DutyCycleOut dutyCycle = new DutyCycleOut(0);
    VoltageOut voltageOut = new VoltageOut(0);
    VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(slot);
    MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(slot);
    MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(slot);
    PositionVoltage positionVoltage = new PositionVoltage(0).withSlot(slot);

    Data<Double> closedLoopSPSignal;
    Data<Double> closedLoopErrorSignal;
    Data<Angle> positionSignal;
    Data<AngularVelocity> velocitySignal;
    Data<AngularAcceleration> accelerationSignal;
    Data<Voltage> voltageSignal;
    Data<Current> currentSignal;

    ControlMode controlMode = ControlMode.DISABLE;

    public TalonFXMotor(TalonFXConfig config) {
        super(config.id, config.canbus.canbus);
        this.config = config;
        name = config.name;
        configMotor();
        setSignals();
        addLog();
        setName(name);
        SmartDashboard.putData(name,this);
        LogManager.log(name + " motor initialized");
    }

    private void configMotor() {
        cfg = new TalonFXConfiguration();
        cfg.CurrentLimits.SupplyCurrentLimit = config.maxCurrent;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = config.maxCurrent;
        cfg.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.ClosedLoopRamps.VoltageClosedLoopRampPeriod = config.rampUpTime;
        cfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = config.rampUpTime;

        cfg.MotorOutput.Inverted = config.inverted ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.Clockwise_Positive;
        cfg.MotorOutput.NeutralMode = config.brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        cfg.MotorOutput.PeakForwardDutyCycle = config.maxVolt / 12.0;
        cfg.MotorOutput.PeakReverseDutyCycle = config.minVolt / 12.0;
        cfg.Feedback.SensorToMechanismRatio = config.motorRatio;
        updatePID(false);
        cfg.Voltage.PeakForwardVoltage = config.maxVolt;
        cfg.Voltage.PeakReverseVoltage = config.minVolt;
        configureMotionMagic(false);


        getConfigurator().apply(cfg);
    }

    private void configureMotionMagic(boolean apply) {
        cfg.MotionMagic.MotionMagicAcceleration = config.maxAcceleration;
        cfg.MotionMagic.MotionMagicCruiseVelocity = config.maxVelocity;
        cfg.MotionMagic.MotionMagicJerk = config.maxJerk;
        if(apply) {
            getConfigurator().apply(cfg.MotionMagic);
            LogManager.log(" motion param " + config.maxVelocity + " , " + config.maxAcceleration + " k=" 
                + cfg.MotionMagic.MotionMagicExpo_kV + ", " + cfg.MotionMagic.MotionMagicExpo_kA);
        }

    }

    private void updatePID(boolean apply) {
        cfg.Slot0.kP = config.pid[0].kP();
        cfg.Slot0.kI = config.pid[0].kI();
        cfg.Slot0.kD = config.pid[0].kD();
        cfg.Slot0.kS = config.pid[0].kS();
        cfg.Slot0.kV = config.pid[0].kV();
        cfg.Slot0.kA = config.pid[0].kA();
        cfg.Slot0.kG = config.pid[0].kG();
        cfg.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        cfg.Slot1.kP = config.pid[1].kP();
        cfg.Slot1.kI = config.pid[1].kI();
        cfg.Slot1.kD = config.pid[1].kD();
        cfg.Slot1.kS = config.pid[1].kS();
        cfg.Slot1.kV = config.pid[1].kV();
        cfg.Slot1.kA = config.pid[1].kA();
        cfg.Slot1.kG = config.pid[1].kG();
        cfg.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        
        cfg.Slot2.kP = config.pid[2].kP();
        cfg.Slot2.kI = config.pid[2].kI();
        cfg.Slot2.kD = config.pid[2].kD();
        cfg.Slot2.kS = config.pid[2].kS();
        cfg.Slot2.kV = config.pid[2].kV();
        cfg.Slot2.kA = config.pid[2].kA();
        cfg.Slot2.kG = config.pid[2].kG();
        cfg.Slot2.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        if(apply) {
            getConfigurator().apply(cfg.Slot0);
            getConfigurator().apply(cfg.Slot1);
            getConfigurator().apply(cfg.Slot2);
        }
    }

    @Override
    public void setName(String name) {
        MotorInterface.super.setName(name);
        this.name = name;
    }

    @SuppressWarnings("unchecked")
    private void setSignals() {
        closedLoopSPSignal = new Data<>(getClosedLoopReference());
        closedLoopErrorSignal = new Data<>(getClosedLoopError());
        positionSignal = new Data<>(getPosition());
        velocitySignal = new Data<>(getVelocity());
        accelerationSignal = new Data<>(getAcceleration());
        voltageSignal = new Data<>(getMotorVoltage());
        currentSignal = new Data<>(getStatorCurrent());

        // BaseStatusSignal.setUpdateFrequencyForAll(50, 
        //     closedLoopSPSignal.getSignal(),
        //     closedLoopErrorSignal.getSignal(),
        //     positionSignal.getSignal(),
        //     velocitySignal.getSignal(),
        //     accelerationSignal.getSignal(),
        //     voltageSignal.getSignal(),
        //     currentSignal.getSignal()
        // );
    }

    @SuppressWarnings("unchecked")
    private void addLog() {
        LogManager.addEntry(name + ": Position, Velocity, Acceleration, Voltage, Current, CloseLoopError, CloseLoopSP",  new StatusSignal[] {
            positionSignal.getSignal(),
            velocitySignal.getSignal(),
            accelerationSignal.getSignal(),
            voltageSignal.getSignal(),
            currentSignal.getSignal(),
            closedLoopErrorSignal.getSignal(),
            closedLoopSPSignal.getSignal(),
            }).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP)
            .withIsMotor().build();
        LogManager.addEntry(name + ": ControlMode", 
            () -> getCurrentControlMode())
            .withLogLevel(LogLevel.LOG_ONLY_NOT_IN_COMP).build();
    }

    public void checkElectronics() {
        int fault = getFaultField().getValue();
        if (fault != 0) {
            LogManager.log(name + " have fault num: " + fault, AlertType.kError);
        }
    }

    /**
     * change the slot of the pid and feed forward.
     * will not work if the slot is null
     * 
     * @param slot the wanted slot between 0 and 2
     */
    public void changeSlot(int slot) {
        if (slot < 0 || slot > 2) {
            LogManager.log("slot is not between 0 and 2", AlertType.kError);
            return;
        }
        this.slot = slot;
        velocityVoltage.withSlot(slot);
        motionMagicVoltage.withSlot(slot);
        motionMagicExpoVoltage.withSlot(slot);
        positionVoltage.withSlot(slot);
    }

    /*
     * set motor to brake or coast
     */
    public void setNeutralMode(boolean isBrake) {
        cfg.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        getConfigurator().apply(cfg.MotorOutput);
    }

    /**
     * set power from 1 to -1 (v/12) no PID/FF
     * 
     * @param power the wanted power between -1 to 1
     */
    public void setDuty(double power) {
        setControl(dutyCycle.withOutput(power));
        if (power == 0){
            controlMode = ControlMode.DISABLE;
        } else {
            controlMode = ControlMode.DUTYCYCLE;
        }
    }

    public void setVoltage(double voltage) {
        setControl(voltageOut.withOutput(voltage));
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
        setControl(velocityVoltage.withVelocity(velocity).withFeedForward(feedForward));
        controlMode = ControlMode.VELOCITY;
    }

    public void setVelocity(double velocity) {
        setVelocity(velocity, 0);
    }

    /**
     * set motion magic with PID and Ff
     * <br>
     * </br>
     * must add to config motion magic configs (vel, acc, jerk[optional])
     * 
     * @param position    the wanted position in meter or radians depending on the
     *                    config
     * @param feedForward wanted feed forward to add to the ks kv ka and kg defaults
     *                    to 0
     */
    public void setMotion(double position, double feedForward) {
        setControl(motionMagicExpoVoltage.withPosition(position).withFeedForward(feedForward));
        controlMode = ControlMode.MOTION;  
    }

    public void setMotion(double position) {
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
  
    public void setPositionVoltage(double position, double feedForward) {
        setControl(positionVoltage.withPosition(position).withFeedForward(feedForward));
        controlMode = ControlMode.POSITION_VOLTAGE;
    }

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

    public int getCurrentControlMode() {
        return controlMode.ordinal();
    }

    public double getCurrentClosedLoopSP() {
        Double value = closedLoopSPSignal.getDouble();
        return value != null ? value : 0.0;
    }
    
    public double getCurrentClosedLoopError() {
        Double value = closedLoopErrorSignal.getDouble();
        return value != null ? value : 0.0;
    }
    
    public double getCurrentPosition() {
        Double value = positionSignal.getDouble();
        return value != null ? value : 0.0;
    }
    
    public double getCurrentVelocity() {
        Double value = velocitySignal.getDouble();
        return value != null ? value : 0.0;
    }
    
    public double getCurrentAcceleration() {
        Double value = accelerationSignal.getDouble();
        return value != null ? value : 0.0;
    }
    
    public double getCurrentAngle() {
        if(config.isRadiansMotor) {
            return MathUtil.angleModulus(getCurrentPosition());
        }
        return 0;
    }
    
    public double getCurrentVoltage() {
        Double value = voltageSignal.getDouble();
        return value != null ? value : 0.0;
    }
    
    public double getCurrentCurrent() {
        Double value = currentSignal.getDouble();
        return value != null ? value : 0.0;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Talon Motor");
        builder.addDoubleProperty("CloseLoopError", this::getCurrentClosedLoopError, null);
        builder.addDoubleProperty("Position", this::getCurrentPosition, null);
        builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
        builder.addDoubleProperty("Acceleration", this::getCurrentAcceleration, null);
        builder.addDoubleProperty("Voltage", this::getCurrentVoltage, null);
        builder.addDoubleProperty("Current", this::getCurrentCurrent, null);
        if(config.isRadiansMotor) {
            builder.addDoubleProperty("Angle", this::getCurrentAngle, null);
        }
        builder.addDoubleProperty("ControlMode", this::getCurrentControlMode, null);
    }
	  
  /**
   * creates a widget in elastic of the pid and ff for hot reload
   * @param slot the slot of the close loop perams (from 0 to 2)
   */
  public void configPidFf(int slot) {

    Command configPidFf = new InstantCommand(()-> {
      SlotConfigs cfg = new SlotConfigs();
      cfg.SlotNumber = slot;
      switch (slot) {
        case 0:
          cfg.kP = config.pid[0].kP();
          cfg.kI = config.pid[0].kI();
          cfg.kD = config.pid[0].kD();
          cfg.kS = config.pid[0].kS();
          cfg.kV = config.pid[0].kV();
          cfg.kA = config.pid[0].kA();
          cfg.kG = config.pid[0].kG();
          break;

        case 1:
          cfg.kP = config.pid[0].kP();
          cfg.kI = config.pid[0].kI();
          cfg.kD = config.pid[0].kD();
          cfg.kS = config.pid[0].kS();
          cfg.kV = config.pid[0].kV();
          cfg.kA = config.pid[0].kA();
          cfg.kG = config.pid[0].kG();
          break;

        case 2:
          cfg.kP = config.pid[0].kP();
          cfg.kI = config.pid[0].kI();
          cfg.kD = config.pid[0].kD();
          cfg.kS = config.pid[0].kS();
          cfg.kV = config.pid[0].kV();
          cfg.kA = config.pid[0].kA();
          cfg.kG = config.pid[0].kG();
          break;
      
        default:
          cfg.kP = config.pid[0].kP();
          cfg.kI = config.pid[0].kI();
          cfg.kD = config.pid[0].kD();
          cfg.kS = config.pid[0].kS();
          cfg.kV = config.pid[0].kV();
          cfg.kA = config.pid[0].kA();
          cfg.kG = config.pid[0].kG();
          break;
      }

      getConfigurator().apply(cfg);
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
      cfg = new TalonFXConfiguration();
      
      cfg.MotionMagic.MotionMagicAcceleration = config.maxAcceleration;
      cfg.MotionMagic.MotionMagicCruiseVelocity = config.maxVelocity;
      cfg.MotionMagic.MotionMagicJerk = config.maxJerk;
      
      getConfigurator().apply(cfg);
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
      setPosition(position);
    }
    public Data<Double> getClosedLoopErrorSignal() {
        return closedLoopErrorSignal;
    }
    public Data<Double> getClosedLoopSPSignal() {
        return closedLoopSPSignal;
    }
    public Data<Angle> getPositionSignal() {
        return positionSignal;
    }
    public Data<AngularVelocity> getVelocitySignal() {
        return velocitySignal;
    }
    public Data<AngularAcceleration> getAccelerationSignal() {
        return accelerationSignal;
    }
    public Data<Voltage> getVoltageSignal() {
        return voltageSignal;
    }
    public Data<Current> getCurrentSignal() {
        return currentSignal;
    }

    public void stop(){
        stopMotor();
        controlMode = ControlMode.DISABLE;
    }
}