package frc.demacia.utils.Mechanisms;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.demacia.utils.Log.LogManager;
import frc.demacia.utils.Motors.MotorInterface;
import frc.demacia.utils.Sensors.SensorInterface;

public class StateBasedMechanism extends BaseMechanism {

    public interface MechanismState {
        double[] getValues();
        String name();
    }

    public String name;

    SendableChooser<MechanismState> stateChooser = new SendableChooser<>();
    public MechanismState state;
    private final MechanismState IDLE_STATE = new MechanismState() {
        @Override 
        public double[] getValues() { 
            return new double[motors != null ? motors.size() : 0]; 
        }
        @Override
        public String name() {
            return "IDLE";
        }
    };

    private final MechanismState TESTING_STATE = new MechanismState() {
        @Override 
        public double[] getValues() { 
            return getTestValues(); 
        }
        @Override
        public String name() {
            return "TESTING";
        }
    };

    protected double[] testValues;

    public StateBasedMechanism(String name, MotorInterface[] motors, SensorInterface[] sensors, Class<? extends MechanismState> enumClass){
        super(name, motors, sensors);
        testValues = new double[motors.length];
        addNT(enumClass);
    }

    private void addNT(Class<? extends MechanismState> enumClass) {
        stateChooser.addOption("TESTING", TESTING_STATE);
        stateChooser.addOption("IDLE", IDLE_STATE);
        stateChooser.addOption("IDLE2", IDLE_STATE);
        for (MechanismState state : enumClass.getEnumConstants()) {
            stateChooser.addOption(state.name(), state);
        }
        stateChooser.onChange(state -> this.state = state);
        SmartDashboard.putData(getName() + "/State Chooser", stateChooser);
    }

    /**
     * Sets the starting state for this mechanism.
     * 
     * @param state Initial state (must implement MechanismState)
     * @return this mechanism for chaining
     */
    public void setStartingOption(MechanismState state){
        if (state == null) {
            LogManager.log("Starting state cannot be null");
            return;
        }

        stateChooser.setDefaultOption(state.name(), state);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleArrayProperty(getName() + "/Test Values", () -> getTestValues(), testValues -> setTestValues(testValues));
    }

    /**
     * Gets the current state.
     * 
     * @return Current state enum
     */
    public void setState(MechanismState state) {
        this.state = state;
    }

    /**
     * Gets the current state.
     * 
     * @return Current state enum
     */
    public MechanismState getState() {
        return state;
    }

    public double[] getTestValues(){
        return testValues != null? testValues : new double[0];
    }

    public void setTestValues(double[] testValues){
        this.testValues = testValues;
    }
}