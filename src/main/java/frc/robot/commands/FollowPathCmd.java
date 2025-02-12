package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FollowPathCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    
    public FollowPathCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    //dan eu te amo ass:jj
    
    @Override
    public void execute() {// Create a list of waypoints from poses. Each pose represents one waypoint.
        Pose2d endPos = new Pose2d(1, 0, new Rotation2d(0));

        System.out.println("Executado");

        swerveSubsystem.createPathFinding(endPos).schedule();
        //AutoBuilder.followPath(path).schedule();
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
    }

}
