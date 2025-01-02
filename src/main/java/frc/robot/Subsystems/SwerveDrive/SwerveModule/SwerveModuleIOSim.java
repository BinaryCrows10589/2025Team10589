package frc.robot.Subsystems.SwerveDrive.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class SwerveModuleIOSim implements SwerveModuleIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1, .025/SwerveModuleConstants.kDriveGearRatio);// 0.025);
    private DCMotorSim turnMotor = new DCMotorSim(DCMotor.getKrakenX60(1), 1, 0.004/SwerveModuleConstants.kTurningGearRatio);

    private PIDController drivePIDController; 
    private PIDController turnPIDController; 
    
    private double desiredVelocityRPM = 0;
    private double desiredPositionRotations = 0;
    private double driveVolts = 0;

    private String swerveModuleName;

    private NetworkTablesTunablePIDConstants driveMotorPIDConstantTuner;
    private NetworkTablesTunablePIDConstants turnMotorPIDConstantTuner;

    public SwerveModuleIOSim(String swerveModuleName) {
        this.swerveModuleName = swerveModuleName;
        configDirvePID();
        configTurnPID();

        this.driveMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("SwerveModule/DrivePIDValues",
            SwerveModuleConstants.kPModuleSIMDrivePIDValue,
            SwerveModuleConstants.kIModuleSIMDrivePIDValue,
            SwerveModuleConstants.kDModuleSIMDrivePIDValue, 0);

        this.turnMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("SwerveModule/TurnPIDValues",
            SwerveModuleConstants.kPModuleSIMTurnPIDValue,
            SwerveModuleConstants.kIModuleSIMTurnPIDValue,
            SwerveModuleConstants.kDModuleSIMTurnPIDValue, 0);
    }

    /**
     * Configures the drive motor PID Controller. 
     */
    private void configDirvePID() {
        this.drivePIDController = new PIDController(SwerveModuleConstants.kPModuleSIMDrivePIDValue,
            SwerveModuleConstants.kIModuleSIMDrivePIDValue,
            SwerveModuleConstants.kDModuleSIMDrivePIDValue, RobotModeConstants.kLoopPeriod);
    }
    
    /**
     * Configures the turn motor PID Controller. 
     */
    private void configTurnPID() {
        this.turnPIDController = new PIDController(SwerveModuleConstants.kPModuleSIMTurnPIDValue,
            SwerveModuleConstants.kIModuleSIMTurnPIDValue,
            SwerveModuleConstants.kDModuleSIMTurnPIDValue, RobotModeConstants.kLoopPeriod);
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentDrivePIDValues = this.driveMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.driveMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.drivePIDController = new PIDController(currentDrivePIDValues[0],
                currentDrivePIDValues[1], currentDrivePIDValues[2], RobotModeConstants.kLoopPeriod);
        }

        double[] currentTurnPIDValues = this.turnMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.turnMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.turnPIDController = new PIDController(currentTurnPIDValues[0],
                currentTurnPIDValues[1], currentTurnPIDValues[2], RobotModeConstants.kLoopPeriod);
        }
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveMotor.update(LOOP_PERIOD_SECS);
        turnMotor.update(LOOP_PERIOD_SECS);
        
        updatePIDValuesFromNetworkTables();

        //driveVolts = MathUtil.clamp(this.drivePIDController.calculate(this.driveMotor.getAngularVelocityRPM(), this.desiredVelocityRPM), -12, 12);
        double turnVolts = MathUtil.clamp(this.turnPIDController.calculate(this.turnMotor.getAngularPositionRotations(), this.desiredPositionRotations), -12, 12);

        inputs.driveMotorRotations = this.driveMotor.getAngularPositionRotations();
        inputs.driveMotorRPM = this.driveMotor.getAngularVelocityRPM();
        inputs.driveMotorSpeedMetersPerSecond = inputs.driveMotorRPM * SwerveModuleConstants.kDriveConversionVelocityFactor;
        inputs.driveMotorDistanceMeters = (inputs.driveMotorRotations / SwerveModuleConstants.kDriveGearRatio) * SwerveModuleConstants.kWheelDistancePerRotation;
        inputs.driveMotorAppliedVolts = driveVolts;
        inputs.driveMotorCurrentAmps = new double[] {this.driveMotor.getCurrentDrawAmps()};

        inputs.turnMotorAbsolutePositionRotations = this.turnMotor.getAngularPositionRotations();
        inputs.turnMotorRelitivePositionRotations = this.turnMotor.getAngularPositionRotations();
        inputs.wheelAngleRelitivePositionRotations = inputs.turnMotorRelitivePositionRotations / SwerveModuleConstants.kTurningGearRatio;
        inputs.turnMotorRPM = this.turnMotor.getAngularVelocityRPM();
        inputs.turnMotorAppliedVolts = turnVolts;
        inputs.turnMotorCurrentAmps = new double[] {this.turnMotor.getCurrentDrawAmps()};
        
        this.driveMotor.setInputVoltage(this.driveVolts);
        this.turnMotor.setInputVoltage(turnVolts);
    }

    
    public void setDesiredModuleVelocityRPM(double desiredRPM) {
        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "DesiredRPM", desiredRPM);

        this.desiredVelocityRPM = desiredRPM;
    }

    @Override
    public void setDesiredModuleDriveVoltage(double desiredVoltage) {
        this.driveVolts = desiredVoltage;
        //this.driveMotor.setInputVoltage(desiredVoltage);
    }


    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {
        double desiredModuleRotations = desiredModuleAngle.getRotations(); 
        double desiredMotorRotation = desiredModuleRotations * SwerveModuleConstants.kTurningGearRatio;

        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "DesiredModuleRotations" , desiredModuleRotations);
        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "DesiredMotorRotations" , desiredMotorRotation);

        this.desiredPositionRotations = desiredModuleRotations * SwerveModuleConstants.kTurningGearRatio;
    }
}
