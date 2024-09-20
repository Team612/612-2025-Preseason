// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.TrajectoryCommands.TrajectoryCreation;
import frc.robot.commands.TrajectoryCommands.FollowNote;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  SwerveDrivePoseEstimator m_DrivePoseEstimator;
  PhotonPoseEstimator m_PhotonPoseEstimator;
  Vision m_vision;
  Drivetrain m_drivetrain;
  private Field2d m_field;
  private boolean updateWithAprilTags;

  
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(323.25);
  private double previousPipelineTimestamp = 0;

  //Matrix Stds for state estimate
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.01);

  //Matrix Stds for vision estimates
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  static PoseEstimator estimator = null;
  
  public PoseEstimator() {
    m_drivetrain = Drivetrain.getInstance();
    m_vision = Vision.getVisionInstance();
    m_field = new Field2d();
    updateWithAprilTags = true;
    SmartDashboard.putData("Field", m_field);


    m_DrivePoseEstimator = new SwerveDrivePoseEstimator(
      Constants.SwerveConstants.swerveKinematics, 
      m_drivetrain.getNavxAngle(), 
      m_drivetrain.getPositions(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
    m_PhotonPoseEstimator = m_vision.getVisionPose();
  }

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }

  public void isUsingAprilTag(boolean b){
    updateWithAprilTags = b;
  }

  private boolean once = true;

  @Override
  public void periodic() {
    m_DrivePoseEstimator.update(m_drivetrain.getNavxAngle(), m_drivetrain.getPositions());

    if(m_PhotonPoseEstimator != null && updateWithAprilTags){
     
      m_PhotonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
      var estimatedPose = estimatedRobotPose.estimatedPose;
     
      // Make sure we have a new measurement, and that it's on the field
      if (m_vision.getCamera().getLatestResult().getBestTarget().getFiducialId() >= 0){
      if (
        // estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
      estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
      && estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
        if (estimatedRobotPose.targetsUsed.size() >= 1) {
        
          for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
            Pose3d targetPose = m_vision.return_tag_pose(target.getFiducialId());
            Transform3d bestTarget = target.getBestCameraToTarget();
            Pose3d camPose = targetPose.transformBy(bestTarget.inverse());            

      //       //checking from the camera to the tag is less than 4
            if (target.getPoseAmbiguity() <= .2) {
              previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
              m_DrivePoseEstimator.addVisionMeasurement(camPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
          }
        } 
      }

        else {
            previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
            m_DrivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }
      }
      });

      
    }
    m_field.setRobotPose(getCurrentPose());
    SmartDashboard.putNumber("PoseEstimator X", getCurrentPose().getX());
     SmartDashboard.putNumber("PoseEstimator Y", getCurrentPose().getY());
     SmartDashboard.putNumber("PoseEstimator Angle", getCurrentPose().getRotation().getDegrees());
  }


  public Pose2d getCurrentPose() {
    return m_DrivePoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    m_DrivePoseEstimator.resetPosition(m_drivetrain.getNavxAngle(), m_drivetrain.getPositions(), newPose);
  }

}