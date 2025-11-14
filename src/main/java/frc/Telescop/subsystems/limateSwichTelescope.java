package frc.Telescop.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Sensors.LimitSwitch;
import frc.demacia.utils.Sensors.LimitSwitchConfig;

public class limateSwichTelescope extends SubsystemBase {

  LimitSwitch limateSwich;
  LimitSwitchConfig config = new LimitSwitchConfig(0, "TelescopeLimit");

  public limateSwichTelescope() {  
      limateSwich = new LimitSwitch(config);
  }

  public boolean getLimateSwitch() {
      return limateSwich.get();
  }

  @Override
  public void periodic() {
     
  }
}
