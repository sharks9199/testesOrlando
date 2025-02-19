package frc.robot.commands.Autos;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.wpilibj2.command.Command;
    import com.pathplanner.lib.auto.AutoBuilder;
    import frc.robot.Constants.AutoConstants;
    import frc.robot.Constants.FieldPoses;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class AutoCollectCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;

    private Command command;
    private Pose2d pose;

    // ============================================================================

    public AutoCollectCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();
        Translation2d currentTranslation = currentPose.getTranslation();

        if (currentTranslation.getDistance(FieldPoses.kCoralBlueLeft.getTranslation()) <
            currentTranslation.getDistance(FieldPoses.kCoralBlueRight.getTranslation())) {
            pose = FieldPoses.kCoralBlueLeft;
            System.out.println("Left!");

        } else {
            System.out.println("Right!");
            pose = FieldPoses.kCoralBlueRight;
        }

        command = AutoBuilder.pathfindToPose(pose, AutoConstants.constraints);
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();

    }

    @Override
    public void end(boolean interrupted) {
        command.end(interrupted);
        System.out.println("Ended");
    }
    
    @Override
    public boolean isFinished() {
        return command.isFinished();
    }
}
