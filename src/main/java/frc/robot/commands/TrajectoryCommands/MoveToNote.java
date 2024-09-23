// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrajectoryCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class MoveToNote extends Command {
  /** Creates a new MoveToNote. */
  private Drivetrain m_drivetrain;
  private Vision m_vision;

  private double initialAngle = 0;
  private double initialPos = 0;
  private double goalAngle = 0;
  private double goalPos = 0;
  public MoveToNote(Drivetrain d, Vision v) {
    m_drivetrain = d;
    m_vision = v;
    addRequirements(d, v);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_drivetrain.getNavxAngle().getDegrees();
    initialPos = m_drivetrain.getEncoderMeters();
    if(m_vision.hasTarget()) {
      goalAngle = -m_vision.getTargetYaw();
      goalPos = m_vision.getNoteRange() - 0.5; //1 m in front of note
    }
    m_drivetrain.drive(new Translation2d(), 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double curPos = goalPos - (Math.abs(m_drivetrain.getEncoderMeters() - initialPos));
    double curAngle = 0;
    if(Math.abs(goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle)) > 0.1) {
      curAngle = goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle);
    }
    double xSpeed = Math.cos(Units.degreesToRadians(initialAngle + goalAngle)) * curPos;
    double ySpeed = Math.sin(Units.degreesToRadians(initialAngle + goalAngle)) * curPos;
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
    return goalPos - (Math.abs(m_drivetrain.getEncoderMeters() - initialPos)) < 0.1;
  }
}