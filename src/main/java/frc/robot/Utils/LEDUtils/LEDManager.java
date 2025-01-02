package frc.robot.Utils.LEDUtils;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import frc.robot.Constants.LEDConstants;

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
        candleConfiguration.disableWhenLOS = true; // This was false last year
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

    /**
     * Sets the LEDs to a "fire animation" based on the parameters passed in.
     * @param brightness Double: Brightness of the LEDs(0-1)
     * @param speed Double: The speed at wich the flame will process(0-1)
     * @param sparkingRate Double: The rate at which sparks are created(0-1)
     * @param coolingRateDouble: The rate at which the fire fades(0-1)
     */
    public static void fireAnimation(double brightness, double speed, double sparkingRate, double coolingRate) {
        Logger.recordOutput("LED/Color", "Fire Animation");
        candle.animate(new FireAnimation(brightness, speed, LEDConstants.kLEDCount, sparkingRate, coolingRate));
    }

    /**
     * Sets the LEDs to do a rainbow animation
     * @param brightness Double: Brightness of the LEDs(0-1)
     * @param travelSpeed Double: The travel speed of the rainbow(0-1)
     */
    public void rainbowAnimation(double brightness, double travelSpeed) {
        Logger.recordOutput("LED/Color", "Rainbow Animation");
        candle.animate(new RainbowAnimation(1, 1, LEDConstants.kLEDCount));
    }

    /**
     * Sets the LEDs to a Twinkle Animation
     * @param rgbw Integer(Length 4): The RGBW values of the LEDs
     * @param travelSpeed Double: The speed at which the Twinkle animation "twingles"
     */
    public void TwinkleAnimation(int[] rgbw, double travelSpeed) { 
        Logger.recordOutput("LED/Color", "Twinkle Animation");
        candle.animate(new TwinkleAnimation(rgbw[0], rgbw[1], rgbw[2], rgbw[3], travelSpeed, LEDConstants.kLEDCount, TwinklePercent.Percent100));
    }


}
