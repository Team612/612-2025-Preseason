package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision implements Subsystem {
    String limeName;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    public Vision(){
        updateData();
    }

    public void updateData() {
        x = tx.getDouble(0.0);
        y = ty.getDouble(0.0);
        area = ta.getDouble(0.0);
    }


    @Override
    public void periodic() {
        updateData();
    }

    public Pose2d getAprilTagPose(){
        LimelightResults results = LimelightHelpers.getLatestResults(limeName);
        if (results.targets_Fiducials.length > 0) {
            LimelightTarget_Fiducial tag = results.targets_Fiducials[0]; 
            double tx = tag.tx;                 
            double ty = tag.ty;                  
            return new Pose2d(tx, ty, new Rotation2d((tag.getYaw()))); 
        }
        return new Pose2d();
    }
}


