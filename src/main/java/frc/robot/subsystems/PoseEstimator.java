package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;

import java.io.IOException;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase {

  final Vision m_vision;
  private final AprilTagFieldLayout aprilTagFieldLayout;
  SwerveDrivePoseEstimator poseEstimator;
    CommandSwerveDrivetrain m_swerve;
    public PoseEstimator() {
        m_swerve = TunerConstants.DriveTrain;
        m_vision = new Vision();
        AprilTagFieldLayout layout;
        try {
          layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
          if (true) { // Condition should be changed to if blue alliance
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
           }
            else {layout.setOrigin(OriginPosition.kRedAllianceWallRightSide);} 
        } catch(IOException e) {
          DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
          layout = null;
        }
        this.aprilTagFieldLayout = layout;
    }
    
      @Override
      public void periodic() {
        setCurrentPose(m_vision.getAprilTagPose());
        poseEstimator.update(
           m_swerve.getRotation(),
          m_swerve.getSwervePoses());
    }
    
    
      public Pose2d getCurrentPose() {
        return poseEstimator.getEstimatedPosition();
      }
    
      public void setCurrentPose(Pose2d newPose) {
        poseEstimator.resetPosition(
            m_swerve.getRotation(),
            m_swerve.getSwervePoses(),
          newPose);
      }

      public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
      }
    
    }