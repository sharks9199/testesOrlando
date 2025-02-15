package frc.robot.commands.Autos;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.path.PathPlannerPath;
    import frc.robot.Constants.FieldPoses;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class AutoCollectCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;

    private Command command;
    private Pose2d alignPose;
    private Pose2d endPose;

    // ============================================================================

    public AutoCollectCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();
        Translation2d currentTranslation = currentPose.getTranslation();

        if (currentTranslation.getDistance(FieldPoses.kCoralBlueAlignLeft.getTranslation()) <
            currentTranslation.getDistance(FieldPoses.kCoralBlueAlignRight.getTranslation())) {
            alignPose = FieldPoses.kCoralBlueAlignLeft;
            endPose = FieldPoses.kCoralBlueLeft;
            System.out.println("Left!");
        } else {
            System.out.println("Right!");
            alignPose = FieldPoses.kCoralBlueAlignRight;
            endPose = FieldPoses.kCoralBlueRight;
        }

        PathPlannerPath path = swerveSubsystem.createPath(currentPose, alignPose, endPose);
        
        command = AutoBuilder.followPath(path);
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
    }
    
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
