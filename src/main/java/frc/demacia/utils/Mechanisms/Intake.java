package frc.demacia.utils.Mechanisms;

/**
 * State-based intake mechanism.
 * 
 * <p>Uses duty cycle control for rollers. States define motor powers.</p>
 * 
 * <p><b>Example State Enum:</b></p>
 * <pre>
 * public enum IntakeStates implements Intake.IntakeState {
 *     OFF(new double[]{0, 0}),
 *     INTAKE(new double[]{0.8, 0.8}),
 *     EJECT(new double[]{-0.5, -0.5});
 *     
 *     private double[] powers;
 *     IntakeStates(double[] powers) { this.powers = powers; }
 *     public double[] getValues() { return powers; }
 * }
 * </pre>
 * 
 * <p><b>Intake Modes:</b></p>
 * <ul>
 *   <li>SENSOR - Run until sensor detects game piece</li>
 *   <li>TIMED - Run for fixed duration</li>
 *   <li>CONTINUOUS - Run until manually stopped</li>
 * </ul>
 */
public class Intake extends StateBasedMechanism<Intake>{

    public Intake(String name) {
        super(name);
        withConsumer(
        (motor, values) -> {
            for (int i = 0; i < motor.length && i < values.length; i++) {
                motor[i].setDuty(values[i]);
            }});
    }
}