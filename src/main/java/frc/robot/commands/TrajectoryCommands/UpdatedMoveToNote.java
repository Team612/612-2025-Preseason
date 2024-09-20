// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrajectoryCommands;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class UpdatedMoveToNote extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final Vision m_vision;
  private final ProfiledPIDController turnController = new ProfiledPIDController(0.6, 0.0, 0.0, new TrapezoidProfile.Constraints(1.0, 3.0)); 
  private int timer = 0;
  private double rotationspeed = 0;
  private final double offset = 0.0; // Adjust this offset as needed
  private double initialAngle = 0;
  private double initialPos = 0;
  private double goalAngle = 0;
  private double goalPos = 0;
  /** Creates a new UpdatedMoveToNote. */
  public UpdatedMoveToNote(Drivetrain d, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = d;
    m_vision = v;
    addRequirements(d, v);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //these are the variables we define for the rest of the code to work
    initialAngle = m_drivetrain.getNavxAngle().getDegrees(); //intial angle
    initialPos = m_drivetrain.getEncoderMeters(); //intital pose: intital encoder position
    if(m_vision.hasTarget()) {
      goalAngle = -m_vision.getTargetYaw(); //the target's angle relative to camera
      goalPos = m_vision.getNoteRange() - 1; //distance to target in meters
    }
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, false);
    turnController.reset(m_drivetrain.getNavxAngle().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_vision.hasTarget()) {
      double distanceToNote = m_vision.getNoteRange() - 1; //1 meter in front of note
      double rotationToNote = -m_vision.getTargetYaw();
      double rotationError = rotationToNote - m_drivetrain.getNavxAngle().getDegrees() + offset; //calculating error
      rotationspeed = turnController.calculate(rotationError, 0); //pid integration
      double translationspeed = 0.5 * distanceToNote;
      m_drivetrain.drive(new Translation2d(distanceToNote, Rotation2d.fromDegrees(m_drivetrain.getNavxAngle().getDegrees())), rotationspeed, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
        m_drivetrain.driveRobotRelative(new Translation2d(), 0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   if (distanceToNote < 1) { 
  //     return true;
  //   }
  //   return false;
  // }
  return m_drivetrain.getEncoderMeters() >= goalPos - 0.5 || timer >= 10;
}
}
