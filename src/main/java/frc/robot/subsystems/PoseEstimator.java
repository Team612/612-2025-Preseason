// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;


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
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class PoseEstimator extends SubsystemBase {
  /** Creates a new PoseEstimator. */
  SwerveDrivePoseEstimator drivePoseEstimator;
  PhotonPoseEstimator photonPoseEstimatorFront;
  PhotonPoseEstimator photonPoseEstimatorBack;
  Vision visionSubsystem;
  Drivetrain driveSubsystem;
  private Field2d fieldLayout;
  Pose2d poseA = new Pose2d();
  Pose2d poseB = new Pose2d();
StructPublisher<Pose2d> publisher;
StructArrayPublisher<Pose2d> arrayPublisher;
  
  public static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.25);
  public static final double FIELD_WIDTH_METERS = Units.inchesToMeters(323.25);
  private double previousPipelineTimestamp = 0;

  //Matrix Stds for state estimate
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.01);

  //Matrix Stds for vision estimates
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  static PoseEstimator estimator = null;
  
  public PoseEstimator() {
    driveSubsystem = Drivetrain.getInstance();
    visionSubsystem = Vision.getVisionInstance();
    fieldLayout = new Field2d();
    SmartDashboard.putData("Field", fieldLayout);


    drivePoseEstimator = new SwerveDrivePoseEstimator(
      Constants.SwerveConstants.swerveKinematics, 
      driveSubsystem.getNavxAngle(), 
      driveSubsystem.getPositions(), 
      new Pose2d(),
      stateStdDevs,
      visionMeasurementStdDevs
    );
    // photonPoseEstimatorFront = visionSubsystem.getVisionPoseFront();
    photonPoseEstimatorBack = visionSubsystem.getVisionPoseBack();

 publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose2d.struct).publish();
arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

  }

  public static PoseEstimator getPoseEstimatorInstance() {
    if (estimator == null) {
      estimator = new PoseEstimator();
    }
    return estimator;
  }


  /*
   pre-condition: poseEstimator != null and camID == 1 or camID == 2
   1 = FRONT APRILTAG CAMERA
   2 = BACK APRILTAG CAMERA
   */

  public void updateEachPoseEstimator(PhotonPoseEstimator poseEstimator, int camID){    
    if(visionSubsystem.getApriltagCamera().getLatestResult().hasTargets()) {
     poseEstimator.update().ifPresent(estimatedRobotPose -> {
      var estimatedPose = estimatedRobotPose.estimatedPose;
     

      // m_DrivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), FIELD_LENGTH_METERS);
     
      // Make sure we have a new measurement, and that it's on the field
      if (visionSubsystem.getApriltagCamera().getLatestResult().getBestTarget().getFiducialId() >= 0){
       
      if (
        estimatedRobotPose.timestampSeconds != previousPipelineTimestamp && 
      estimatedPose.getX() >= 0.0 && estimatedPose.getX() <= FIELD_LENGTH_METERS
      && estimatedPose.getY() >= 0.0 && estimatedPose.getY() <= FIELD_WIDTH_METERS) {
        if (estimatedRobotPose.targetsUsed.size() >= 1) {
        
          for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
            Pose3d targetPose = visionSubsystem.return_tag_pose(target.getFiducialId());
            Transform3d bestTarget = target.getBestCameraToTarget();
            Pose3d camPose;
            //Adding vision measurements from the center of the robot to the apriltag. Back camera should already be inverted
            // camPose = targetPose.transformBy(bestTarget.inverse().plus(visionSubsystem.getRobotToCam().inverse()));  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 
            camPose = targetPose.transformBy(bestTarget.inverse().plus(new Transform3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0,Units.degreesToRadians(35),Math.PI))));  //.plus(new Transform3d(robotToCam, new Rotation3d(0,0,0))); 

          //checking the tags ambiguity. The lower the ambiguity, the more accurate the pose is
            if (target.getPoseAmbiguity() <= .2) {
              previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
              drivePoseEstimator.addVisionMeasurement(camPose.toPose2d(), estimatedRobotPose.timestampSeconds);
            }
          }
        } 
      }

        else {
            previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
            drivePoseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }
      }
      }
      );
    }

  }

  private boolean once = true;

  @Override
  public void periodic() {
    
    //updates the drivePoseEstimator with with Navx angle and current wheel positions
    drivePoseEstimator.update(driveSubsystem.getNavxAngle(), driveSubsystem.getPositions());


    //update each individual pose estimator
    // if (photonPoseEstimatorFront != null){
    //   updateEachPoseEstimator(photonPoseEstimatorFront, 1);
    // } 
    if(photonPoseEstimatorBack != null){
      updateEachPoseEstimator(photonPoseEstimatorBack, 2);
    }

    publisher.set(drivePoseEstimator.getEstimatedPosition());
    //arrayPublisher.set(new Pose3d[] {poseA, poseB});
    
    //updates the robot pose in the field simulation
    fieldLayout.setRobotPose(getCurrentPose());

    // SmartDashboard.putNumber("PoseEstimator X", getCurrentPose().getX());
    // SmartDashboard.putNumber("PoseEstimator Y", getCurrentPose().getY());
    // SmartDashboard.putNumber("PoseEstimator Angle", getCurrentPose().getRotation().getDegrees());
    // SmartDashboard.putNumber("PoseEstimator Radians", getCurrentPose().getRotation().getRadians());
  }


  public Pose2d getCurrentPose() {
    return drivePoseEstimator.getEstimatedPosition();
  }

  public void setCurrentPose(Pose2d newPose) {
    drivePoseEstimator.resetPosition(driveSubsystem.getNavxAngle(), driveSubsystem.getPositions(), newPose);
  }

}