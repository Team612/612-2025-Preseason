// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrajectoryCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AlignSpeakerManual extends Command {
  private Drivetrain m_drivetrain;
  private Vision m_vision;
  private double initialAngle;
  private double goalAngle;
  private double initialMeters;
  private double goalMeters;
  /** Creates a new AlignSpeakerManual. */
  public AlignSpeakerManual(Drivetrain d, Vision v) {
    m_drivetrain = d;
    m_vision = v;
    addRequirements(d, v);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d tagPose = m_vision.getTagPose();
    initialAngle = m_drivetrain.getNavxAngle().getDegrees() - tagPose.getRotation().getDegrees();
    initialMeters = m_drivetrain.getEncoderMeters();
    goalMeters = Math.sqrt( Math.pow(tagPose.getX(), 2) + Math.pow(tagPose.getY(), 2) );
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      goalAngle = 180;
    } else {
      goalAngle = 0;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curPos = goalMeters - (Math.abs(m_drivetrain.getEncoderMeters() - initialMeters));
    double curAngle = 0;
    if(Math.abs(goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle)) > 0.1) {
      curAngle = goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle);
    }
    double xSpeed = Math.cos(Units.degreesToRadians(initialAngle)) * curPos;
    double ySpeed = Math.sin(Units.degreesToRadians(initialAngle)) * curPos;
    m_drivetrain.drive(new Translation2d(xSpeed, ySpeed), Units.degreesToRadians(curAngle) * 2, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(new Translation2d(), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return goalMeters - (Math.abs(m_drivetrain.getEncoderMeters() - initialMeters)) < 0.1;  }
}
