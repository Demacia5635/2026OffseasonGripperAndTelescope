package frc.demacia.utils.Motors;

/** 
 * Class to hold all Spark motor configuration
 * Applicable to REV Spark Max/Flex
 *  */
public class SparkFlexConfig extends BaseMotorConfig<SparkFlexConfig> {

    // SparkMotorType motorType = SparkMotorType.SparkMax;

    /** 
     * Constructor
     * @param id - canbus ID
     * @param name - name of motor for logging
     */
    public SparkFlexConfig(int id, String name) {
        super(id, name);
        motorClass = MotorControllerType.SparkFlex;
    }

    public SparkFlexConfig(int id, String name, SparkMaxConfig config) {
        this(id,name);
        copyBaseFields(config);
    }
}