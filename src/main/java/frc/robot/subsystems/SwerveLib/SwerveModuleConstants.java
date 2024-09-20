package frc.robot.subsystems.SwerveLib;

import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int cancoderID;
  public final Rotation2d desiredAngle;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param canCoderID
   * @param desiredAngle
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int canCoderID, Rotation2d desiredAngle) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.cancoderID = canCoderID;
    this.desiredAngle = desiredAngle;
  }
}