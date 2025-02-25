package frc.robot.commands.Autos;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.wpilibj2.command.Command;
    import com.pathplanner.lib.auto.AutoBuilder;
    import frc.robot.Constants.AutoConstants;
    import frc.robot.Constants.FieldPoses;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class AutoScoreCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private Command command;
    // ============================================================================

    public AutoScoreCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Pose2d currentPose = swerveSubsystem.getPoseEstimator();
        // Translation2d currentTranslation = currentPose.getTranslation();
        
        // // Inicializa as variáveis de comparação
        // Pose2d[] gridPoses = FieldPoses.gridPoses;

        // Pose2d closestPose = null;
        // double minDistance = Double.MAX_VALUE;
        // double distance;
        
        // // Itera sobre as poses e calcula a distância uma vez
        // for (Pose2d pose : gridPoses) {
        //     distance = currentTranslation.getDistance(pose.getTranslation()); // Calcula a distância
            
        //     if (distance < minDistance) { // Atualiza se encontrar uma pose mais próxima
        //         minDistance = distance;
        //         closestPose = pose;
        //     }
        // }

        // System.out.println("Closest Pose: " + closestPose);

        command = AutoBuilder.pathfindToPose(FieldPoses.kGrid12, AutoConstants.constraints);
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
