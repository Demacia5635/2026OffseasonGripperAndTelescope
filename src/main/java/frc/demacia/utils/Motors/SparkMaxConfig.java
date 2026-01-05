package frc.demacia.utils.Motors;

/** 
 * Class to hold all Spark motor configuration
 * Applicable to REV Spark Max/Flex
 *  */
public class SparkMaxConfig extends BaseMotorConfig<SparkMaxConfig> {

    // SparkMotorType motorType = SparkMotorType.SparkMax;

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */
    public SparkMaxConfig(int id, String name) {
        super(id, name);
        motorClass = MotorControllerType.SparkMax;
    }

    public SparkMaxConfig(int id, String name, BaseMotorConfig<?> config) {
        this(id,name);
        copyBaseFields(config);
    }
}