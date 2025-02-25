package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {

    public double getID(String limelightName) {
        return LimelightHelpers.getFiducialID(limelightName);
    }

    public PoseEstimate getMeasurement(double angle, String limelightName){
        LimelightHelpers.SetRobotOrientation(limelightName, angle, 0.0, 0.0, 0.0, 0.0, 0.0);
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        
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
        return LimelightHelpers.getTargetPose_RobotSpace(limelightName);
    }

    public double getTX(String limelightName){
        return getBotPose(limelightName)[0];
    }
    
    public double getTZ(String limelightName){
        return getBotPose(limelightName)[2];
    }
    
    public double getRY(String limelightName){
        return getBotPose(limelightName)[4];
    }

    @Override
    public void periodic() {
        if (getID(LimelightConstants.LimelightReefLeft) > 1) {
            double tx = getTX(LimelightConstants.LimelightReefLeft);
            double tz = getTZ(LimelightConstants.LimelightReefLeft);
            double angularError = getRY(LimelightConstants.LimelightReefLeft);

            SmartDashboard.putNumber("TX Left", tx);
            SmartDashboard.putNumber("TZ Left", tz);
            SmartDashboard.putNumber("angularError Left", angularError);
        }

        if (getID(LimelightConstants.LimelightReefRight) > 1) {
            double tx = getTX(LimelightConstants.LimelightReefRight);
            double tz = getTZ(LimelightConstants.LimelightReefRight);
            double angularError = getRY(LimelightConstants.LimelightReefRight);

            SmartDashboard.putNumber("TX Right", tx);
            SmartDashboard.putNumber("TZ Right", tz);
            SmartDashboard.putNumber("angularError Right", angularError);
        }
        
    }

}
