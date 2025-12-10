package frc.demacia.utils.Sensors;
public class UltraSonicSensorConfig extends AnalogSensorConfig<UltraSonicSensorConfig> {
    int pingChannel;
    int echoChannel;
    public UltraSonicSensorConfig(int echoChannel, int pingChannel, String name) {
        super(echoChannel, name);
        this.pingChannel = pingChannel;
    }
}