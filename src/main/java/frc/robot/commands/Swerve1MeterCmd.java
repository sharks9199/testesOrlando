package frc.robot.commands;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.math.kinematics.SwerveModuleState;
    import edu.wpi.first.wpilibj2.command.Command;
    import java.util.function.Supplier;
    import frc.robot.Constants.DriveConstants;
    import frc.robot.subsystems.SwerveSubsystem;
    import edu.wpi.first.math.controller.PIDController;
// ============================================================================

public class Swerve1MeterCmd extends Command {
    
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private PIDController xSpeedPID;
    private double xSpeed, position;
    // ============================================================================

    public Swerve1MeterCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        xSpeedPID = new PIDController(0.8, 0, 0);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        System.out.println("1 meter running");
        position = swerveSubsystem.getPoseEstimator().getX();

        xSpeed = xSpeedPID.calculate(position, 2);

        ChassisSpeeds chassisSpeeds;
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, 0, 0, swerveSubsystem.getRotation2d());

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        swerveSubsystem.setModuleStates(moduleStates);

    }

    @Override
    public void end(boolean interrupted) {
        // Para os módulos
        swerveSubsystem.stopModules();
    }

}
