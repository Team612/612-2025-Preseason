// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands.AutoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

public class AutoIntake extends Command {
  /** Creates a new MoveToNote. */
  private Drivetrain m_drivetrain;
  private Vision m_vision;
  private Intake m_intake;

  private double initialAngle = 0;
  private double goalAngle = 0;
  private double count = 0;
  public AutoIntake(Drivetrain d, Vision v, Intake i) {
    m_drivetrain = d;
    m_vision = v;
    m_intake = i;
    addRequirements(d, v, i);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = m_drivetrain.getNavxAngle().getDegrees();
    if(m_vision.hasTarget()) {
      goalAngle = -m_vision.getTargetYaw();
    }
    m_drivetrain.driveRobotRelative(new Translation2d(), 0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intake.getIRSensor() > 0.3) {
      count++;
    } else {
      count = 0;
    }
    m_intake.moveRollers(-Constants.IntakeConstants.rollerSpeedIntake);
    double curAngle = 0;
    if(Math.abs(goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle)) > 0.1) {
      curAngle = goalAngle - (m_drivetrain.getNavxAngle().getDegrees() - initialAngle);
    }
    double xSpeed = Math.cos(Units.degreesToRadians(initialAngle + goalAngle));
    double ySpeed = Math.sin(Units.degreesToRadians(initialAngle + goalAngle));
    m_drivetrain.drive(new Translation2d(xSpeed / 2, ySpeed / 2), Units.degreesToRadians(curAngle) / 2, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.driveRobotRelative(new Translation2d(0, 0), 0, false);
    m_intake.moveRollers(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return count >= 2;
  }
}