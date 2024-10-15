// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveLib.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveConstants {
    public static final double stickDeadband = 0.1;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(27);
    public static final double wheelBase = Units.inchesToMeters(27);
    public static final double wheelDiameter = Units.inchesToMeters(3.75);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150.0 / 7.0); // 12.8:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.58453; //0.22005
    public static final double driveKV = 2.40269; //2.74490
    public static final double driveKA = 0;

     //have to tune manually
     public static final double kPXController = 5; // ~ 1cm error
     public static final double kPYController = 1;
     public static final double kPThetaController = 4.5; 

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = Math.PI;
    public static final double maxAcceleration = 1;
    public static final double maxAngularAcceleration = Math.PI/6;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /*
     *** INSTRUCTIONS TO SET OFFSETS ***
     * PHYSICALLY SET ALL MODULES TO 0 (facing front)
     * WRITE CANCODER VALUE AS OFFSET FOR EACH MODULE
     * MAKE SURE THAT EACH MODULE IS MOVING IN THE RIGHT DIRECTION
     * IF OPPOSITE DIRECTION, THEN ADD 180 TO OFFSET (or subtract 180 if offset becomes greater than 360)
    */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 0;
      public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(223); //43
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 1;
      public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(344); //344
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 2;
      public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(87);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 6;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;
      public static final Rotation2d desiredAngle = Rotation2d.fromDegrees(73.5);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, desiredAngle);
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static int kGunnerControllerPort = 1;
  }

  public static class VisionConstants{
    public static String cameraNameAprilTagFront = "Front";
    public static String cameraNameAprilTagBack = "Back";
    public static String cameraNameObject = "Front";

    //constraints
    public static final TrapezoidProfile.Constraints ThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI, Math.PI);
    public static final TrapezoidProfile.Constraints PControllerConstraints =
        new TrapezoidProfile.Constraints(1.5,1.5);
    public static final TrapezoidProfile.Constraints StrafeControllerConstaints = 
        new TrapezoidProfile.Constraints(1.5, 1.5);
    
    //controllers
    public static final ProfiledPIDController rotationController = 
      new ProfiledPIDController(.01, 0, 0, ThetaControllerConstraints);
    public static final ProfiledPIDController forwardController = 
      new ProfiledPIDController(.5, 0, 0, PControllerConstraints);
    public static final ProfiledPIDController strafeController = 
      new ProfiledPIDController(.4, 0, 0, StrafeControllerConstaints);

    //other camera constants
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(13.5);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(14.25);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(12);
    public static final double GOAL_RANGE_METERS = 1;
    
    public static final Transform3d CAMERA_TO_Robot = new Transform3d();
  }
  
  // Intake constants
  public static class IntakeConstants{
    public static final String pivotUpKey = "Pivot Speed";
    public static final String rollerInKey = "Roller Speed";
    
    public static final int pivotID = 9;
    public static final int rollerID = 10;
    public static final double intakeUpSpeed = 0.3;
    public static final double intakeDownSpeed = -0.3;
    public static final double rollerSpeedIntake = 0.5;
    public static double rollerSpeedOuttake = 0.7;
    public static final int IRport = 0;
  }


  // Shooter constants
  public static class ShooterConstants{
    public static final String leftSpeedKey = "Left Speed";
    public static final String rightSpeedKey = "Right Speed";

    public static final int shooterLeftID = 17;
    public static final int shooterRightID = 16;
    public static double shooterLeftSpeedSpeaker = -.6;
    public static double shooterRightSpeedSpeaker = -.6; //0.9
    public static double shooterLeftSpeedAmp = -0.3;
    public static double shooterRightSpeedAmp = 0.3;
  }

  public static class ClimbConstants {
    public static final int climbLeftID = 13;
    public static final int climbRightID = 14;
    public static final double climbLeftSpeed = 0.3;
    public static final double climbRightSpeed = -0.3;
  }
}
