package frc.robot.commands.Autos;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.controller.PIDController;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.Constants.AutoConstants;
    import frc.robot.Constants.LimelightConstants;
    import frc.robot.subsystems.LimelightSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class AutoAlignLeftCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private PIDController angularAlignPIDController, xAlignPIDController, zAlignPIDController;
    private double tx, tz, angularError, xSpeed, zSpeed, angularSpeed;
    // ============================================================================

    public AutoAlignLeftCmd(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;

        angularAlignPIDController = new PIDController(AutoConstants.kPLimelightAlignAngular, 0, 0.00);
        angularAlignPIDController.setTolerance(0.5);

        xAlignPIDController = new PIDController(AutoConstants.kPLimelightAlignX, 0, 0.00);
        xAlignPIDController.setTolerance(0.01);
        
        zAlignPIDController = new PIDController(AutoConstants.kPLimelightAlignZ, 0, 0.00);
        zAlignPIDController.setTolerance(0.01);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        tx = limelightSubsystem.getTX(LimelightConstants.LimelightReefRight);
        tz = limelightSubsystem.getTZ(LimelightConstants.LimelightReefRight);
        angularError = limelightSubsystem.getRY(LimelightConstants.LimelightReefRight);

        xSpeed = xAlignPIDController.calculate(tx, 0.13);
        zSpeed = zAlignPIDController.calculate(tz, 0.35);
        angularSpeed = angularAlignPIDController.calculate(angularError, 6);

        zSpeed = Math.min(Math.max(zSpeed, -1.3), 1.3);
        xSpeed = Math.min(Math.max(xSpeed, -0.5), 0.5);
        angularSpeed = Math.min(Math.max(angularSpeed, -0.5), 0.5);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("zSpeed", zSpeed);
        SmartDashboard.putNumber("angularSpeed", angularSpeed);

        swerveSubsystem.setRobotOrientedSpeed(-zSpeed, xSpeed, angularSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotOrientedSpeed(0, 0, 0);
        swerveSubsystem.stopModules();
        System.out.println("Alinhado");
    }

    @Override
    public boolean isFinished() {
        return (xAlignPIDController.atSetpoint() && zAlignPIDController.atSetpoint() && angularAlignPIDController.atSetpoint());
    }

}