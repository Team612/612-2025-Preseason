// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
//shruthi da best
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class TrajectoryConfiguration extends SubsystemBase {
  private static TrajectoryConfiguration trajectoryConfig;

  private PoseEstimator m_PoseEstimator = PoseEstimator.getPoseEstimatorInstance();
  private Drivetrain m_Drivetrain = Drivetrain.getInstance();
  /** Creates a new TrajectoryConfiguration. */
  public TrajectoryConfiguration() {
    AutoBuilder.configureHolonomic(
                m_PoseEstimator::getCurrentPose, // Robot pose supplier
                m_PoseEstimator::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
                m_Drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                m_Drivetrain::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        Units.inchesToMeters(13.02), // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_Drivetrain // Reference to this subsystem to set requirements
        );
  }

  public Command followPathGui(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        
        return new FollowPathHolonomic(
                path,
                m_PoseEstimator::getCurrentPose, // Robot pose supplier
                m_Drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                m_Drivetrain::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_Drivetrain // Reference to this subsystem to set requirements
        );
    }

    public Command followPathManual(PathPlannerPath path) {
        return new FollowPathHolonomic(
                path,
                m_PoseEstimator::getCurrentPose, // Robot pose supplier
                m_Drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                m_Drivetrain::autoDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                m_Drivetrain // Reference to this subsystem to set requirements
        );
    }

    public static TrajectoryConfiguration getInstance(){
      if (trajectoryConfig == null){
         trajectoryConfig = new TrajectoryConfiguration();
      }
      return trajectoryConfig;
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Current Pose", m_PoseEstimator.getCurrentPose().getX() + ", " + m_PoseEstimator.getCurrentPose().getY());
  }
}