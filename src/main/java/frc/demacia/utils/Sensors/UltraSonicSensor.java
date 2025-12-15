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

    
    private final int average_Window = 5;
    private final Queue<Double> samples = new ArrayDeque<>();

    public double getAverage() {
        double current = getRangeMeters();
        
        if(samples.size() < 5) {
            samples.add(current);
            return current;
        }

        samples.remove();
        samples.add(current);
        return calcAvg();

    }
    private double calcAvg(){
        double sum = 0;
        int c =0;
        for(double val : samples){
            sum+=val;
            c++;
        }
        return c != 0 ? sum / c : 0;

    }

    private void addLog() {
        LogManager.addEntry(name + "range", () -> getRangeMeters(), 3);
        LogManager.addEntry(name + "avg range", () -> getAverage(), 3);

    }
}
