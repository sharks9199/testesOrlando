package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FollowPathCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public FollowPathCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements();
    }

    @Override
    public void initialize() {
        System.out.println("Follow Path Started!");
    }
    
    @Override
    public void execute() {// Create a list of waypoints from poses. Each] pose represents one waypoint.
    Pose2d currentPose = swerveSubsystem.getPoseEstimator();
      
    // The rotation component in these poses represents the direction of travel
    Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
    Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d());

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
    PathPlannerPath path = new PathPlannerPath(
    waypoints, 
    new PathConstraints(
        1.0, 1.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
    ),
    null, // Ideal starting state can be null for on-the-fly paths
    new GoalEndState(0.0, currentPose.getRotation())
    );

    // Prevent this path from being flipped on the red alliance, since the given positions are already correct
    path.preventFlipping = true;

    AutoBuilder.followPath(path).schedule();
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Acabou");
    }

}
