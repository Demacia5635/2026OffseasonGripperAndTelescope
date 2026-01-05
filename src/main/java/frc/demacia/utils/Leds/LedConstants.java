package frc.demacia.utils.Leds;

/**constants for leds */
public class LedConstants {
  /**the size of every strip for every port */
  public static final int LENGTH = 14;
  /**the port of the leds */
  public static final int PORT = 2;

  /**
   * the blink time between what is color and what is off <br></br>
   * {@code Timer.getFPGATimestamp() % BLINK_TIME != 0 ? color : Color.kBlack}
   */
  public static final double BLINK_TIME = 3;
}