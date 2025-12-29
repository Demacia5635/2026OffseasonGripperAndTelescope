package frc.demacia.utils.Motors;

import com.ctre.phoenix6.CANBus;

/**
 * Abstract base class for motor configurations.
 */
public abstract class BaseMotorConfig<T extends BaseMotorConfig<T>> {
    
    public static enum Canbus { 
        Rio("rio"), 
        CANIvore("canivore");
    
        public final CANBus canbus;
        private Canbus(String name) {
            this.canbus = new CANBus(name);
        }
    } 

    public static enum MotorControllerType {
        TalonFX {
            @Override
            public MotorInterface create(BaseMotorConfig<?> config) {
                return new TalonFXMotor((TalonFXConfig) config);
            }
        },
        TalonSRX {
            @Override
            public MotorInterface create(BaseMotorConfig<?> config) {
                return new TalonSRXMotor((TalonSRXConfig) config);
            }
        },
        SparkMax {
            @Override
            public MotorInterface create(BaseMotorConfig<?> config) {
                return new SparkMaxMotor((SparkMaxConfig) config);
            }
        },
        SparkFlex {
            @Override
            public MotorInterface create(BaseMotorConfig<?> config) {
                return new SparkFlexMotor((SparkFlexConfig) config);
            }
        };

        public abstract MotorInterface create(BaseMotorConfig<?> config);
    }

    public int id;
    public Canbus canbus = Canbus.Rio;
    public MotorControllerType motorClass = MotorControllerType.TalonFX;
    public String name;

    public double maxVolt = 12;
    public double minVolt = -12;
    public double maxCurrent = 40;
    public double rampUpTime = 0.3;

    public boolean brake = true;
    public double motorRatio = 1;
    public boolean inverted = false;

    public double maxVelocity = 0;
    public double maxAcceleration = 0;
    public double maxJerk = 0;
    public double maxPositionError = 0.5;

    public CloseLoopParam[] pid = {new CloseLoopParam(), new CloseLoopParam(), new CloseLoopParam(), new CloseLoopParam()};

    public boolean isMeterMotor = false;
    public boolean isRadiansMotor = false;

    public double kv2 = 0;
    public double kSin = 0;
    public double posToRad = 0;

    public BaseMotorConfig(int id, String name) {
        this.id = id;
        this.name = name;
    }

    public BaseMotorConfig(int id, String name, Canbus canbus) {
        this(id, name);
        this.canbus = canbus;
    }
    
    public MotorControllerType getMotorClass() {
        return motorClass;
    }

    @SuppressWarnings("unchecked")
    public T withMotorClass(MotorControllerType motorClass) {
        this.motorClass = motorClass;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withVolts(double maxVolt) {
        this.maxVolt = maxVolt;
        this.minVolt = -maxVolt;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withBrake(boolean brake) {
        this.brake = brake;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withInvert(boolean invert) {
        this.inverted = invert;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withRampTime(double rampTime) {
        this.rampUpTime = rampTime;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withMeterMotor(double gearRatio, double diameter) {
        motorRatio = gearRatio / (diameter * Math.PI);
        isMeterMotor = true;
        isRadiansMotor = false;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withRadiansMotor(double gearRatio) {
        motorRatio = gearRatio / (Math.PI * 2);
        isMeterMotor = false;
        isRadiansMotor = true;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withMaxPositionError(double maxPositionError) {
        this.maxPositionError = maxPositionError;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withCurrent(double maxCurrent) {
        this.maxCurrent = maxCurrent;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withMotionParam(double maxVelocity, double maxAcceleration, double maxJerk) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxJerk = maxJerk;
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withFeedForward(double kv2, double ksin, double posToRad) {
        this.kv2 = kv2;
        this.kSin = ksin;
        this.posToRad = posToRad;
        return (T) this;
    }

    public T withPID(double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        return withPID(0, kp, ki, kd, ks, kv, ka, kg);
    }
    
    @SuppressWarnings("unchecked")
    public T withPID(int slot, double kp, double ki, double kd, double ks, double kv, double ka, double kg) {
        if (slot >= 0 && slot < pid.length) {
            pid[slot] = new CloseLoopParam(kp, ki, kd, ks, kv, ka, kg);
        }
        return (T) this;
    }

    @SuppressWarnings("unchecked")
    public T withCanbus(Canbus canbus) {
        this.canbus = canbus;
        return (T) this;
    }

    protected void copyBaseFields(BaseMotorConfig<?> other) {
        this.canbus = other.canbus;
        this.maxVolt = other.maxVolt;
        this.minVolt = other.minVolt;
        this.maxCurrent = other.maxCurrent;
        this.rampUpTime = other.rampUpTime;
        this.brake = other.brake;
        this.motorRatio = other.motorRatio;
        this.inverted = other.inverted;
        this.kv2 = other.kv2;
        this.kSin = other.kSin;
        this.posToRad = other.posToRad;
        this.maxAcceleration = other.maxAcceleration;
        this.maxVelocity = other.maxVelocity;
        this.maxJerk = other.maxJerk;
        this.pid[0] = (other.pid[0]);
        this.pid[1] = (other.pid[1]);
        this.pid[2] = (other.pid[2]);
        this.pid[3] = (other.pid[3]);
        this.maxPositionError = other.maxPositionError;
        this.isMeterMotor = other.isMeterMotor;
        this.isRadiansMotor = other.isRadiansMotor;
   }
}