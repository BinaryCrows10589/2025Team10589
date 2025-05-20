// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Constants.CameraConstants.VisionConstants;
import frc.robot.Constants.GenericConstants.ControlConstants;
import frc.robot.Constants.GenericConstants.FieldConstants;
import frc.robot.Constants.GenericConstants.RobotModeConstants;
import frc.robot.CrowMotion.Library.CMPathGenResult;
import frc.robot.CrowMotion.Library.CMPathGenerator;
import frc.robot.CrowMotion.UserSide.CMAutonPoint;
import frc.robot.CrowMotion.UserSide.CMEvent;
import frc.robot.CrowMotion.UserSide.CMRotation;
import frc.robot.CrowMotion.UserSide.CMRotation.RotationDirrection;
import frc.robot.Utils.GeneralUtils.PercentError;
import frc.robot.Utils.GeneralUtils.Tolerance;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesChangableValue;
import frc.robot.Utils.LEDUtils.LEDManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    private NetworkTablesChangableValue autonDebugMode = new NetworkTablesChangableValue("RobotMode/AutonDebugMode", RobotModeConstants.kAutonDebugMode);
    private ArrayList<CompletableFuture<CMPathGenResult>> CMPaths = new ArrayList<CompletableFuture<CMPathGenResult>>();
    private boolean isDone1 = false;
    private boolean isDone2 = false;
    private boolean isDone3 = false;
        /**
         * This function is run when the robot is first started up and should be used for any
         * initialization code.
         */
        @SuppressWarnings("resource")
        @Override
        public void robotInit() {
            //DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);

            // BuiltConstatns is generated when the project is built. No imports or anything is needed if it is red underlined just build the project
            // Record metadata
            Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
            Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
            Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
            Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
            Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
            Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

            switch (BuildConstants.DIRTY) {
                case 0:
                    Logger.recordMetadata("GitDirty", "All changes committed");
                    break;
                case 1:
                    Logger.recordMetadata("GitDirty", "Uncomitted changes");
                    break;
                default:
                    Logger.recordMetadata("GitDirty", "Unknown");
                    break;
            }

            

            // Set up data receivers & replay source
            switch (RobotModeConstants.currentMode) {
                case REAL:
                    // Running on a real robot, log to a USB stick ("/U/logs")
                    Logger.addDataReceiver(new WPILOGWriter());
                    Logger.addDataReceiver(new NT4Publisher());
                    new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
                    break;
                case SIM:
                    // Running a physics simulator, log to NT
                    Logger.addDataReceiver(new NT4Publisher());
                    break;
                case REPLAY:
                    // Replaying a log, set up replay source
                    setUseTiming(true); // Run as fast as possible
                    String logPath = LogFileUtil.findReplayLog();
                    Logger.setReplaySource(new WPILOGReader(logPath));
                    Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                    break;
            }

            // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
            Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

            // Initalized the LEDs
            LEDManager.init(); 


            // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
            // autonomous chooser on the dashboard.
            robotContainer = new RobotContainer();
            checkDriverStationUpdate();
            LEDManager.setSolidColor(new int[] {255, 0, 255});
        }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test. 
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        RobotModeConstants.kAutonDebugMode = (boolean)this.autonDebugMode.getChangableValueOnNetworkTables();
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        this.robotContainer.updateButtonBoardInputs();
        checkDriverStationUpdate();
        this.robotContainer.periodic();        
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {    
        if(RobotModeConstants.kAutonDebugMode) {
            this.robotContainer = new RobotContainer();
        }
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    public Pose2d getTargetedAutonPosition() {
        Pose2d robotPosition = robotContainer.getRobotPosition();
        /*boolean isStartingOnOtherBarge = Tolerance.inTolorance(robotPosition.getY(), AutonPointManager.kOtherAllianceBargeStartPosition.getAutonPoint().getY(), 1.0);
        return isStartingOnOtherBarge  ? AutonPointManager.kOtherAllianceBargeStartPosition.getAutonPoint() : AutonPointManager.kOwnAllianceBargeStartPosition.getAutonPoint();*/
       if(Tolerance.inTolorance(robotPosition.getY(), AutonPointManager.kOwnAllianceBargeStartPosition.getAutonPoint().getY(), 1)) {
            return AutonPointManager.kOwnAllianceBargeStartPosition.getAutonPoint();
        }   
        return AutonPointManager.kOtherAllianceBargeStartPosition.getAutonPoint();
    }

    @Override
    public void disabledPeriodic() {
        // Indicators to help us line up with the target starting position
        Pose2d actualPosition = robotContainer.getRobotPosition();
        Pose2d desiredPosition = getTargetedAutonPosition();
        Logger.recordOutput("LED/DesiredPosition", desiredPosition);

        boolean[] tolerances = Tolerance.inTolerancePose2d(desiredPosition, actualPosition, ControlConstants.robotStartPositionTolorence);
        
        Logger.recordOutput("LED/ActualRotationRad", actualPosition.getRotation().getRadians());
        Logger.recordOutput("LED/DesiredRotationRad", desiredPosition.getRotation().getRadians());

        LEDManager.setAxisIndicators(
            PercentError.getPercentError(actualPosition.getX(), desiredPosition.getX()),
            PercentError.getPercentError(actualPosition.getY(), desiredPosition.getY()),
            PercentError.getPercentError(actualPosition.getRotation(), desiredPosition.getRotation()),

            tolerances[0],
            tolerances[1],
            tolerances[2]);
    }

    /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
    @Override
    public void autonomousInit() {
        checkDriverStationUpdate();
        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
        autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}


    @Override
    public void teleopInit() {
        VisionConstants.updateVision = true;
        checkDriverStationUpdate();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
        autonomousCommand.cancel();
        }
        
        this.CMPaths.add(
        CMPathGenerator.generateCMPathAsync(
                "TestBezier",
                new CMAutonPoint[] {
                    new CMAutonPoint(1, 1),
                    new CMAutonPoint(5, 2),
                    new CMAutonPoint(7, 3.5),
                    new CMAutonPoint(6.5, 4.5),
                    new CMAutonPoint(6, 4)
                },
                this.robotContainer.getRobotPosition().getRotation().getDegrees(),
                new CMRotation[] {
                    new CMRotation(20, RotationDirrection.NEGITIVE, 1)
                },
                new CMEvent[] {
                    new CMEvent("TestEvent0", () -> {
                        Logger.recordOutput("CrowMotion/EventCalled0", true);
                    }, 0),
                    new CMEvent("TestEvent1", () -> {
                        Logger.recordOutput("CrowMotion/EventCalled1", true);
                    }, 1)
                }
            )
        );
            
        this.CMPaths.add(
        CMPathGenerator.generateCMPathAsync(
                "TestLinearFromBot",
                new CMAutonPoint[] {
                    new CMAutonPoint(5, 2),
                },
                this.robotContainer.getRobotPosition().getRotation().getDegrees(),
                new CMRotation[] {
                    new CMRotation(20, RotationDirrection.NEGITIVE, 1)
                },
                new CMEvent[] {
                    new CMEvent("TestEvent0", () -> {
                        Logger.recordOutput("CrowMotion/EventCalled0", true);
                    }, 0),
                    new CMEvent("TestEvent1", () -> {
                        Logger.recordOutput("CrowMotion/EventCalled1", true);
                    }, 1)
                }
            )
        );
         
        this.CMPaths.add(
        CMPathGenerator.generateCMPathAsync(
                "TestLinear",
                new CMAutonPoint[] {
                    new CMAutonPoint(2, 2),
                    new CMAutonPoint(10, 5),
                },
                this.robotContainer.getRobotPosition().getRotation().getDegrees(),
                new CMRotation[] {
                    new CMRotation(20, RotationDirrection.NEGITIVE, 1)
                },
                new CMEvent[] {
                    new CMEvent("TestEvent0", () -> {
                        Logger.recordOutput("CrowMotion/EventCalled0", true);
                    }, 0),
                    new CMEvent("TestEvent1", () -> {
                        Logger.recordOutput("CrowMotion/EventCalled1", true);
                    }, 1)
                }
            )
        );
        
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if(this.CMPaths.get(0).isDone() && !isDone1) {
            this.isDone1 = true;
            Logger.recordOutput("CrowMotion/PathGenTimeBezier", (System.currentTimeMillis() - RobotModeConstants.startPathGenTime)/1000);
            Logger.recordOutput("CrowMotion/TestBezier", this.CMPaths.get(0).getNow(new CMPathGenResult(null, null)).loggingPoints);
        }

        if(this.CMPaths.get(1).isDone() && !isDone2) {
            this.isDone2 = true;
            Logger.recordOutput("CrowMotion/PathGenTimeLinearFromBot", (System.currentTimeMillis() - RobotModeConstants.startPathGenTime)/1000);
            Logger.recordOutput("CrowMotion/TestLinearFromBot", this.CMPaths.get(1).getNow(new CMPathGenResult(null, null)).loggingPoints);
        }


        if(this.CMPaths.get(2).isDone() && !isDone3) {
            this.isDone3 = true;
            Logger.recordOutput("CrowMotion/PathGenTimeLinear", (System.currentTimeMillis() - RobotModeConstants.startPathGenTime)/1000);
            Logger.recordOutput("CrowMotion/TestLinear", this.CMPaths.get(2).getNow(new CMPathGenResult(null, null)).loggingPoints);
        }
        
        //this.robotContainer.driveSubsystem().drive(4.311, 0 ,0);
        //RobotProfilingUtil.ProfileMaxPossibleRotationalVelocityDPS.profileMaxPossibleRotationalVelocityDPS();
        //RobotProfilingUtil.ProfileMaxPossibleTranslationalVelocityMPSMaxPossibleAverageSwerveModuleMPS.profileMaxPossibleTranslationalVelocityMPSAndMaxPossibleAverageSwerveModuleMPS();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {}

    private void checkDriverStationUpdate() {
        Optional<Alliance> currentAllianceFromDriverStation = DriverStation.getAlliance();
        Alliance currentAlliance = currentAllianceFromDriverStation.orElse(Alliance.Blue);
        // If we have data, and have a new alliance from last time
        
        if (DriverStation.isDSAttached() && currentAlliance != FieldConstants.alliance) {
            FieldConstants.alliance = currentAlliance;
            RobotModeConstants.isBlueAlliance = currentAlliance == Alliance.Blue;
            RobotModeConstants.hasAllianceChanged = true;
            this.robotContainer.updateAlliance();
        }
    }
}
