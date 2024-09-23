// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TurnWhenLeft extends Command {
  /** Creates a new TurnWhenLeft. */
  private final Drivetrain m_drivetrain;
  public TurnWhenLeft(Drivetrain d) {
    m_drivetrain = d;
    addRequirements(d);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.zeroGyro();
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.driveRobotRelative(new Translation2d(0, 0), -Math.PI / 4, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_drivetrain.getNavxAngle().getDegrees() <= -40;
  }
}
