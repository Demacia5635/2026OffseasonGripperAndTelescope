package frc.demacia.utils.Motors;


/** 
 * Class to hold all Talon FX/SRX configuration
 * Applicable to Phoenix 6
 *  */
public class TalonFXConfig extends BaseMotorConfig<TalonFXConfig> {

    /** 
     * Constructor
     * @param id - canbus ID
     * @param canbus - Name of canbus
     * @param name - name of motor for logging
     */
    public TalonFXConfig(int id, Canbus canbus, String name) {
        super(id, name, canbus);
        motorClass = MotorControllerType.TalonFX;
    }

    public TalonFXConfig(int id, String name, TalonFXConfig config) {
        super(id, name);
        copyBaseFields(config);
    }
}