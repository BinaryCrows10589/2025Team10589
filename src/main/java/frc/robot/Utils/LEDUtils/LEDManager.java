package frc.robot.Utils.LEDUtils;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import frc.robot.Constants.MechanismConstants.LEDConstants;

public class LEDManager {

    private static CANdle candle;
    private static CANdleConfiguration candleConfiguration;

    /**
     * Must be called above the creation of the RobotContainer in the Robot.java robotInit() method but after the all of the Advantage Kit set up. This initalized the LEDs. 
     * This configures the Candle
     */
    public static void init() {
        candle = new CANdle(LEDConstants.kLEDPortID);
        candleConfiguration = new CANdleConfiguration();
        candleConfiguration.statusLedOffWhenActive = false;
        candleConfiguration.disableWhenLOS = false; // This was false last year
        candleConfiguration.stripType = LEDStripType.RGB;
        candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(candleConfiguration);
    }

    /**
     * Sets the LED light strips on the robot to one solid color
     * @param rgb Integer Array(length 3), RGB color to set the LEDs to. 
     */
    public static void setSolidColor(int[] rgb) {
        Logger.recordOutput("LED/Color", rgb);
        candle.setLEDs(rgb[0], rgb[1], rgb[2]);
    }

    private static int[] getAxisIndicatorLEDColor(boolean inTolerance, double percentError) {
        return inTolerance ? LEDConstants.kLEDColorCorrect : ((percentError) > 0 ? LEDConstants.kLEDColorHigh : LEDConstants.kLEDColorLow);
    }
    
    public static void setAxisIndicators(double percentErrorX, double percentErrorY, double percentErrorRot,
                                         boolean inToleranceX, boolean inToleranceY, boolean inToleranceRot) {

        int[] colorX = getAxisIndicatorLEDColor(inToleranceX, percentErrorX);
        int[] colorY = getAxisIndicatorLEDColor(inToleranceY, percentErrorY);
        int[] colorRot = getAxisIndicatorLEDColor(inToleranceRot, percentErrorRot);

        // Set blank ones first
        //setSolidColor(new int[] {0, 0, 0});

        // Set three axes up
        candle.setLEDs(colorX[0], colorX[1], colorX[2], 255, 28, 10);
        candle.setLEDs(colorY[0], colorY[1], colorY[2], 0, 18, 10);
        candle.setLEDs(colorRot[0], colorRot[1], colorRot[2], 0, 8, 10);
        
        Logger.recordOutput("LED/AxisIndicatorXColor", colorX);
        Logger.recordOutput("LED/AxisIndicatorYColor", colorY);
        Logger.recordOutput("LED/AxisIndicatorRotColor", colorRot);


    }

    public static void test() {
        candle.setLEDs(255, 255, 255);
    }

    /**
     * Sets the LEDs to a "fire animation" based on the parameters passed in.
     * @param brightness Double: Brightness of the LEDs(0-1)
     * @param speed Double: The speed at wich the flame will process(0-1)
     * @param sparkingRate Double: The rate at which sparks are created(0-1)
     * @param coolingRateDouble: The rate at which the fire fades(0-1)
     */
    public static void fireAnimation(double brightness, double speed, double sparkingRate, double coolingRate) {
        Logger.recordOutput("LED/Color", "Fire Animation");
        //candle.animate(new FireAnimation(brightness, speed, LEDConstants.kLEDCount, sparkingRate, coolingRate));
    }

    /**
     * Sets the LEDs to do a rainbow animation
     * @param brightness Double: Brightness of the LEDs(0-1)
     * @param travelSpeed Double: The travel speed of the rainbow(0-1)
     */
    public void rainbowAnimation(double brightness, double travelSpeed) {
        Logger.recordOutput("LED/Color", "Rainbow Animation");
        //candle.animate(new RainbowAnimation(1, 1, LEDConstants.kLEDCount));
    }

    /**
     * Sets the LEDs to a Twinkle Animation
     * @param rgbw Integer(Length 4): The RGBW values of the LEDs
     * @param travelSpeed Double: The speed at which the Twinkle animation "twingles"
     */
    public void TwinkleAnimation(int[] rgbw, double travelSpeed) { 
        Logger.recordOutput("LED/Color", "Twinkle Animation");
     //   candle.animate(new TwinkleAnimation(rgbw[0], rgbw[1], rgbw[2], rgbw[3], travelSpeed, LEDConstants.kLEDCount, TwinklePercent.Percent100));
    }


}
