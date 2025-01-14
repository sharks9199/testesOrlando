package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightSubsystem extends SubsystemBase {
    private String LimelightID = "limelight-main"; 
    public final LimelightHelpers limelight = new LimelightHelpers();
    
    public Pose2d getRobotPose(){
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightID);
        return new Pose2d(limelightMeasurement.pose.getX(), limelightMeasurement.pose.getY(), limelightMeasurement.pose.getRotation());
    }

    public double getPoseX(){
        return getRobotPose().getX();
    }

    public double getPoseY(){
        return getRobotPose().getY();
    }

    public Rotation2d getPoseRotation2d(){
        return getRobotPose().getRotation();
    }

    public void setPipeline(int pipeline){
        LimelightHelpers.setPipelineIndex(LimelightID, pipeline);
    }

    public double getTX(){
        return LimelightHelpers.getTX(LimelightID);
    }

    public double getTY(){
        return LimelightHelpers.getTY(LimelightID);
    }

    public double getTZ(){
        return LimelightHelpers.getTZ(LimelightID);
    }

    public double getRY(){
        return LimelightHelpers.getRY(LimelightID);
    }

    @Override
    public void periodic() {
    }
}
