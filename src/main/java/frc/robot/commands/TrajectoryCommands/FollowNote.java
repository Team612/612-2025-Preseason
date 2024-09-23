// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrajectoryCommands;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Vision;

public class FollowNote extends Command {
  private final Drivetrain driveSystem;
  private final PoseEstimator poseEstimatorSystem;
  private final TrajectoryCreation m_traj;
  private final Vision m_vision;
  private final double translation;
  private Transform2d notespace;

  private Command controllerCommand = Commands.none();

  /** Creates a new RunOnTheFly. */
  public FollowNote(Drivetrain d, PoseEstimator p, TrajectoryCreation traj, Vision v,
                    double y) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSystem = d;
    poseEstimatorSystem = p;
    m_traj = traj;
    m_vision = v;
    translation = y;

    addRequirements(d, p);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerPath path = m_traj.noteOnTheFly(poseEstimatorSystem, m_vision,driveSystem);
    if (path == null) {
      System.out.println("NO TARGETS");
      end(true);
    }
    else {
    controllerCommand = AutoBuilder.followPath(path);
    controllerCommand.initialize();
    }

    
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    controllerCommand.execute();
    System.out.println(" Current Pose: " + poseEstimatorSystem.getCurrentPose().getY() + " Speed, " + driveSystem.getStates()[1].speedMetersPerSecond);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("--------------------DONE------------------");
    controllerCommand.end(interrupted);
    System.out.println(poseEstimatorSystem.getCurrentPose().getX());
    System.out.println(poseEstimatorSystem.getCurrentPose().getY());

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controllerCommand.isFinished();
  }
}