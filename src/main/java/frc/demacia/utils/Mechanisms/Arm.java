package frc.demacia.utils.Mechanisms;

/**
 * State-based arm mechanism.
 * 
 * <p>Uses angle control for precise positioning. States define target angles.</p>
 * 
 * <p><b>Example State Enum:</b></p>
 * <pre>
 * public enum ArmStates implements Arm.ArmState {
 *     STOWED(new double[]{Math.toRadians(0)}),
 *     INTAKE(new double[]{Math.toRadians(45)}),
 *     SCORE_HIGH(new double[]{Math.toRadians(120)});
 *     
 *     private double[] angles;
 *     ArmStates(double[] angles) { this.angles = angles; }
 *     public double[] getValues() { return angles; }
 * }
 * </pre>
 */
public class Arm extends StateBasedMechanism<Arm>{

    public Arm(String name) {
        super(name);
        withConsumer(
        (motor, values) -> {
            for (int i = 0; i < motor.length && i < values.length; i++) {
                motor[i].setAngle(values[i]);
            }});
    }
}
