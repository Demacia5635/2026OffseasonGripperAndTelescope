package frc.demacia.utils.Sensors;
import edu.wpi.first.wpilibj.Ultrasonic;
import frc.demacia.utils.Log.LogManager;
import java.util.ArrayDeque;
import java.util.Queue;
public class UltraSonicSensor extends Ultrasonic implements AnalogSensorInterface{
    public UltraSonicSensor( UltraSonicSensorConfig config) {
        super(config.pingChannel, config.echoChannel);
        this.config = config;
        name = config.name;
        addLog();
		LogManager.log(name + " ultrasonic initialized");
    }
    String name;
    UltraSonicSensorConfig config;
    public double get() {
        return getRangeMeters();
    }


    public String getName() {
        return config.name;
    }

    @Override
    public void ping() {
        super.ping();
    }

    public double getRangeMeters() {
        return getRangeMM() / 1000.0 ;
    }

    
    private static final int average_Window = 5;
    private final Queue<Double> samples = new ArrayDeque<>();
    private double sum = 0;

    public double getAverage() {
        double current = getRangeMeters();
        samples.add(current);
        sum += current;

        if (samples.size() > average_Window) {
            sum -= samples.remove();
        }
        if(samples.size() == 0) return current;
        return sum / samples.size();
    }

    private void addLog() {
        LogManager.addEntry(name + "range", () -> getRangeMeters(), 3);
        LogManager.addEntry(name + "avg range", () -> getAverage(), 3);

    }
}
