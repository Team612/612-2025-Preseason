// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrajectoryCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class AlignAmp extends Command {
  private final Vision m_vision;
  private final PoseEstimator poseEstimatorSystem;
  private final TrajectoryCreation m_traj;

  private Command controllerCommand = Commands.none();

  /** Creates a new RunOnTheFly. */
  public AlignAmp(PoseEstimator p, TrajectoryCreation traj, Vision v) {
    // Use addRequirements() here to declare subsystem dependencies.
    poseEstimatorSystem = p;
    m_traj = traj;
    m_vision = v;
    addRequirements(v, p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerPath path = m_traj.alignAmp(poseEstimatorSystem, m_vision);
    controllerCommand = AutoBuilder.followPath(path);
    controllerCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controllerCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}