package frc.demacia.utils.Mechanisms;

import java.util.function.Supplier;

import frc.demacia.utils.LookUpTable;

/**
 * State-based shooter mechanism.
 * 
 * <p>Uses feed forward for velocity. lookup table define target velocity.</p>
 * 
 * </pre>
 */
public class Shooter extends BaseMechanism<Shooter>{

    public Shooter(String name, LookUpTable lookUpTable, Supplier<Double> posSupplier) {
        super(name);
        withConsumer(
        (motor, values) -> {
            for (int i = 0; i < motor.length && i < values.length; i++) {
                motor[i].setVelocity(values[i]);
            }});
        withLookUpTable(lookUpTable, posSupplier);
    }
}
