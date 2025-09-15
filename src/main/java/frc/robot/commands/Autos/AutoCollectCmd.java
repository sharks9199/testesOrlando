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
    private Pose2d collectPose;
    // ============================================================================

    public AutoCollectCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Obtemos a pose atual do robô
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();
        Translation2d currentTranslation = currentPose.getTranslation();

        // Calcula a distância do robô para as duas posições de referência
        double distanceToLeft = currentTranslation.getDistance(FieldPoses.kCoralBlueLeft.getTranslation());
        double distanceToRight = currentTranslation.getDistance(FieldPoses.kCoralBlueRight.getTranslation());

        // Verifica qual das posições (esquerda ou direita) está mais próxima e retorna a pose
        if (distanceToLeft < distanceToRight) {
            System.out.println("Left!");
            collectPose = FieldPoses.kCoralBlueLeft;
        } else {
            System.out.println("Right!");
            collectPose = FieldPoses.kCoralBlueRight;
        }
    }

    @Override
    public void execute() {
        AutoBuilder.pathfindToPose(collectPose, AutoConstants.constraints).schedule();

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Ended");
    }
    
}
