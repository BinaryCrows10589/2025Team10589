package frc.robot.Utils.AutonUtils;

import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

public class WPILIBTrajectoryConfig {
    public double[] translationPIDValues;
    public double[] rotaitonPIDValues;
    public double maxTranslationSpeedMPS;
    public double maxTranslationAccelerationMPS;
    public double maxRotationSpeedRadsPerSecond;
    public double maxRotationAccelerationRadsPerSecond;

    public WPILIBTrajectoryConfig(double[] translationPIDValues,
        double[] rotaitonPIDValues,
        double maxTranslationSpeedMPS, 
        double maxTranslationAccelerationMPS, 
        double maxRotationSpeedRadsPerSecond, 
        double maxRotationAccelerationRadsPerSecond) {
        
        this.translationPIDValues = translationPIDValues;
        this.rotaitonPIDValues = rotaitonPIDValues;
        this.maxTranslationSpeedMPS = maxTranslationSpeedMPS;
        this.maxTranslationAccelerationMPS = maxTranslationAccelerationMPS;
        this.maxRotationSpeedRadsPerSecond = maxRotationSpeedRadsPerSecond;
        this.maxRotationAccelerationRadsPerSecond = maxRotationAccelerationRadsPerSecond;
    }
}
