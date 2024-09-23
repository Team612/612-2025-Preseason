// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.SwerveLib.SwerveModule;

/** Add your docs here. */
public class ShuffleboardTabs {
    private ShuffleboardTab driverTab;
    private ShuffleboardTab drivetrainTab;
    private ShuffleboardTab intakeTab;
    private ShuffleboardTab poseEstimatorTab;
    private ShuffleboardTab shooterTab;
    private ShuffleboardTab visionTab;
    private ShuffleboardTab climbTab;

    /* Driver Entries */

    /* Debug Entries */
    private GenericEntry mod1_velocity;
    private GenericEntry mod2_velocity;
    private GenericEntry mod3_velocity;
    private GenericEntry mod4_velocity;

    private GenericEntry mod1_angle;
    private GenericEntry mod2_angle;
    private GenericEntry mod3_angle;
    private GenericEntry mod4_angle;


    private GenericEntry cancoder1_angle;
    private GenericEntry cancoder2_angle;
    private GenericEntry cancoder3_angle;
    private GenericEntry cancoder4_angle;

    private GenericEntry navxAngle;

    private GenericEntry limitForward;
    private GenericEntry limitBackward;
    private GenericEntry irSensor;

    private GenericEntry poseEstimatorX;
    private GenericEntry poseEstimatorY;
    private GenericEntry poseEstimatorAngle;
    private GenericEntry poseEstimatorRadians;

    private GenericEntry shooterLeftCurrent;
    private GenericEntry shooterRightCurrent;

    private GenericEntry hasCalibrationFront;
    private GenericEntry hasCalibrationBack;
    private GenericEntry hasTag;
    private GenericEntry tagInSight;
    
    private GenericEntry limitLeftClimbUp;
    private GenericEntry limitLeftClimbDown;
    private GenericEntry limitRightClimbUp;
    private GenericEntry limitRightClimbDown;

    /* Subsystems */
    private Drivetrain driveSubsystem;
    private Intake intakeSubsystem;
    private Shooter shooterSubsystem;
    private Vision visionSubsystem;
    private Climb climbSubsystem;
    private frc.robot.subsystems.PoseEstimator poseEstimatorSubsystem;
    private SwerveModuleState[] states;
    private SwerveModule[] mods;


    public void initButton(){
        /* Init subsystems */
        driveSubsystem = Drivetrain.getInstance();
        intakeSubsystem = Intake.getInstance();
        shooterSubsystem = Shooter.getInstance();
        visionSubsystem = Vision.getVisionInstance();
        climbSubsystem = Climb.getInstance();
        poseEstimatorSubsystem = frc.robot.subsystems.PoseEstimator.getPoseEstimatorInstance();
        states = driveSubsystem.getStates();
        mods = driveSubsystem.getModules();

        /* Init tabs */
        driverTab = Shuffleboard.getTab("Driver Tab");
        drivetrainTab = Shuffleboard.getTab("Drivetrain Tab");
        intakeTab = Shuffleboard.getTab("Intake Tab");
        shooterTab = Shuffleboard.getTab("Shooter Tab");
        visionTab = Shuffleboard.getTab("Vision Tab");
        climbTab = Shuffleboard.getTab("Climb Tab");
        
        poseEstimatorTab = Shuffleboard.getTab("Pose Estimator Tab");

        /* Init entries */

        mod1_velocity = drivetrainTab.add("Mod 1 Velocity", 0.0).getEntry();
        mod2_velocity = drivetrainTab.add("Mod 2 Velocity", 0.0).getEntry();
        mod3_velocity = drivetrainTab.add("Mod 3 Velocity", 0.0).getEntry();
        mod4_velocity = drivetrainTab.add("Mod 4 Velocity", 0.0).getEntry();

        mod1_angle = drivetrainTab.add("Mod 1 Angle", 0.0).getEntry();
        mod2_angle = drivetrainTab.add("Mod 2 Angle", 0.0).getEntry();
        mod3_angle = drivetrainTab.add("Mod 3 Angle", 0.0).getEntry();
        mod4_angle = drivetrainTab.add("Mod 4 Angle", 0.0).getEntry();

        cancoder1_angle = drivetrainTab.add("Cancoder 1 Angle", 0.0).getEntry();
        cancoder2_angle = drivetrainTab.add("Cancoder 2 Angle", 0.0).getEntry();
        cancoder3_angle = drivetrainTab.add("Cancoder 3 Angle", 0.0).getEntry();
        cancoder4_angle = drivetrainTab.add("Cancoder 4 Angle", 0.0).getEntry();

        navxAngle = drivetrainTab.add("Navx Angle", 0.0).getEntry();

        limitForward = intakeTab.add("Limit Forward?", false).getEntry();
        limitBackward = intakeTab.add("Limit Back?", false).getEntry();
        irSensor = intakeTab.add("IR Sensor", 0.0).getEntry();

        poseEstimatorX = poseEstimatorTab.add("Pose Estimator X", 0.0).getEntry();
        poseEstimatorY = poseEstimatorTab.add("Pose Estimator Y", 0.0).getEntry();
        poseEstimatorAngle = poseEstimatorTab.add("Pose Estimator Angle",0.0).getEntry();
        poseEstimatorRadians = poseEstimatorTab.add("Pose Estimator Radians",0.0).getEntry();

        shooterLeftCurrent = shooterTab.add("Shooter Left Current", 0.0).getEntry();
        shooterRightCurrent = shooterTab.add("Shooter Right Current", 0.0).getEntry();

        limitLeftClimbUp = climbTab.add("Climb Left Up", false).getEntry();
        limitLeftClimbDown = climbTab.add("Climb Left Down", false).getEntry();

        limitRightClimbUp = climbTab.add("Climb Right Up", false).getEntry();
        limitRightClimbDown = climbTab.add("Climb Right Down", false).getEntry();


        // hasCalibrationFront = visionTab.add("Front Camera Calibrated?", false).getEntry();
        // hasCalibrationBack = visionTab.add("Back Camera Calibrated?", false).getEntry();
        // hasTag = visionTab.add("Tag in sight?", false).getEntry();
        // tagInSight = visionTab.add("Tag ID (currently in sight)", -1).getEntry();

        
    }

    public void updateButtons(){
        mod1_velocity.setDouble(states[0].speedMetersPerSecond);
        mod2_velocity.setDouble(states[1].speedMetersPerSecond);
        mod3_velocity.setDouble(states[2].speedMetersPerSecond);
        mod4_velocity.setDouble(states[3].speedMetersPerSecond);

        mod1_angle.setDouble(states[0].angle.getDegrees());
        mod2_angle.setDouble(states[1].angle.getDegrees());
        mod3_angle.setDouble(states[2].angle.getDegrees());
        mod4_angle.setDouble(states[3].angle.getDegrees());

        cancoder1_angle.setDouble(mods[0].getCanCoder().getDegrees());
        cancoder2_angle.setDouble(mods[1].getCanCoder().getDegrees());
        cancoder3_angle.setDouble(mods[2].getCanCoder().getDegrees());
        cancoder4_angle.setDouble(mods[3].getCanCoder().getDegrees());

        navxAngle.setDouble(driveSubsystem.getNavxAngle().getDegrees());

        limitForward.setBoolean(intakeSubsystem.getIntakeLimitStateForward());
        limitBackward.setBoolean(intakeSubsystem.getIntakeLimitStateReverse());
        irSensor.setDouble(intakeSubsystem.getIRSensor());

        poseEstimatorX.setDouble(poseEstimatorSubsystem.getCurrentPose().getX());
        poseEstimatorY.setDouble(poseEstimatorSubsystem.getCurrentPose().getY());
        poseEstimatorAngle.setDouble(poseEstimatorSubsystem.getCurrentPose().getRotation().getDegrees());
        poseEstimatorRadians.setDouble(poseEstimatorSubsystem.getCurrentPose().getRotation().getRadians());

        shooterLeftCurrent.setDouble(shooterSubsystem.getCurrent());
        shooterRightCurrent.setDouble(shooterSubsystem.getCurrent());

        // hasCalibrationFront.setBoolean(visionSubsystem.hasCalibrationFront());
        // hasCalibrationBack.setBoolean(visionSubsystem.hasCalibrationBack());
        // hasTag.setBoolean(visionSubsystem.hasTag());
        // tagInSight.setInteger(visionSubsystem.getTagID());

        limitLeftClimbUp.setBoolean(climbSubsystem.getLeftLimitStateUp());
        limitLeftClimbDown.setBoolean(climbSubsystem.getLeftLimitStateDown());
        limitRightClimbUp.setBoolean(climbSubsystem.getRightLimitStateUp());
        limitRightClimbDown.setBoolean(climbSubsystem.getRightLimitStateUp());
    }







}



