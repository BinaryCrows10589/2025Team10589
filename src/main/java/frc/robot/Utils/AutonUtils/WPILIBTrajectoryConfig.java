package frc.robot.Utils.AutonUtils;


public class WPILIBTrajectoryConfig {
    public double[] translationPIDValues;
    public double[] rotationPIDValues;
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
        this.rotationPIDValues = rotaitonPIDValues;
        this.maxTranslationSpeedMPS = maxTranslationSpeedMPS;
        this.maxTranslationAccelerationMPS = maxTranslationAccelerationMPS;
        this.maxRotationSpeedRadsPerSecond = maxRotationSpeedRadsPerSecond;
        this.maxRotationAccelerationRadsPerSecond = maxRotationAccelerationRadsPerSecond;
    }
}
