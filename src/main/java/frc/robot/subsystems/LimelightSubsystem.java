package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightSubsystem extends SubsystemBase {

    public double getID(String limelightName) {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public PoseEstimate getMeasurement(double angle, String limelightName){
        LimelightHelpers.SetRobotOrientation(limelightName, angle, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        
        return limelightMeasurement;
    }
    
    public Pose2d getRobotPose(String limelightName){
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        return new Pose2d(limelightMeasurement.pose.getX(), limelightMeasurement.pose.getY(), limelightMeasurement.pose.getRotation());
    }

    public double getPoseX(String limelightName){
        return getRobotPose(limelightName).getX();
    }

    public double getPoseY(String limelightName){
        return getRobotPose(limelightName).getY();
    }
    
    public Rotation2d getPoseRotation2d(String limelightName){
        return getRobotPose(limelightName).getRotation();
    }
    
    public double getPipeline(String limelightName){
        return LimelightHelpers.getCurrentPipelineIndex(limelightName);
    }

    public void setPipeline(int pipeline, String limelightName){
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    public double getTA(String limelightName){
        return LimelightHelpers.getTA(limelightName);
    }

    public double getx(String limelightName){
        return LimelightHelpers.getTX(limelightName);
    }

    public double gety(String limelightName){
        return LimelightHelpers.getTY(limelightName);
    }

    public double[] getBotPose(String limelightName){
        if (getID(limelightName) > 1){
            return LimelightHelpers.getTargetPose_RobotSpace(limelightName);
        }

        return new double[] {0.0, 0.0, 0.0, 0.0, 0.0};
    }

    public double getTX(String limelightName){
        double[] botPose = getBotPose(limelightName);

        if (botPose.length > 0) {
            return botPose[0];
        } else {
            return 0.0; 
        }
    }
    
    public double getTZ(String limelightName){
        double[] botPose = getBotPose(limelightName);

        if (botPose.length > 0) {
            return botPose[2];
        } else {
            return 0.0; 
        }
    }
    
    public double getRY(String limelightName){
        double[] botPose = getBotPose(limelightName);

        if (botPose.length > 0) {
            return botPose[4];
        } else {
            return 0.0; 
        }
    }

    @Override
    public void periodic() {
    }

}
