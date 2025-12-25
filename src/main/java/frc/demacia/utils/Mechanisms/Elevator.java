package frc.demacia.utils.Mechanisms;

/**
 * State-based elevator mechanism.
 * 
 * <p>Uses motion-profiled position control for smooth movement.
 * States define target heights.</p>
 * 
 * <p><b>Example State Enum:</b></p>
 * <pre>
 * public enum ElevatorStates implements Elevator.ElevatorState {
 *     BOTTOM(new double[]{0.0}),
 *     MID(new double[]{0.5}),
 *     TOP(new double[]{1.2});
 *     
 *     private double[] heights;
 *     ElevatorStates(double[] heights) { this.heights = heights; }
 *     public double[] getValues() { return heights; }
 * }
 * </pre>
 */
public class Elevator extends StateBasedMechanism<Elevator>{

    public Elevator(String name) {
        super(name);
        withConsumer(
        (motor, values) -> {
            for (int i = 0; i < motor.length && i < values.length; i++) {
                motor[i].setMotion(values[i]);
            }});
    }
}
