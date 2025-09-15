package frc.robot.commands.Autos;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.controller.PIDController;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.Constants.AutoConstants;
    import frc.robot.Constants.LEDConstants;
    import frc.robot.Constants.LimelightConstants;
    import frc.robot.Constants.climbConstants;
    import frc.robot.subsystems.ClimbSubsystem;
    import frc.robot.subsystems.LEDSubsystem;
    import frc.robot.subsystems.LimelightSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class AutoAlignLeftCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final ClimbSubsystem climbSubsystem;
    private PIDController angularAlignPIDController, xAlignPIDController, zAlignPIDController;
    private double tx, tz, angularError, xSpeed, zSpeed, angularSpeed, zSetpoint;
    // ============================================================================

    public AutoAlignLeftCmd(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem, 
                LEDSubsystem ledSubsystem, ClimbSubsystem climbSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.climbSubsystem = climbSubsystem;

        angularAlignPIDController = new PIDController(AutoConstants.kPLimelightAlignAngular, 0, 0.00);
        angularAlignPIDController.setTolerance(1);

        xAlignPIDController = new PIDController(3.7, 0, 0);
        xAlignPIDController.setTolerance(0.015);
        
        zAlignPIDController = new PIDController(AutoConstants.kPLimelightAlignZ, 0, 0.00);
        zAlignPIDController.setTolerance(0.04);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() { 
        zSetpoint = 0.42;
        climbConstants.clawSetpoint = 1.7;
        LEDConstants.turboEffect = false;
        limelightSubsystem.setPipeline(2, LimelightConstants.LimelightReefRight);
        
    }
    
    @Override
    public void execute() {
        limelightSubsystem.setPipeline(2, LimelightConstants.LimelightReefRight);
        ledSubsystem.setPattern(LEDConstants.blinkingBlue);

        tx = limelightSubsystem.getTX(LimelightConstants.LimelightReefRight);
        tz = limelightSubsystem.getTZ(LimelightConstants.LimelightReefRight);
        angularError = limelightSubsystem.getRY(LimelightConstants.LimelightReefRight);

        xSpeed = xAlignPIDController.calculate(tx, 0.0);
        zSpeed = zAlignPIDController.calculate(tz, 0.55);
        angularSpeed = angularAlignPIDController.calculate(angularError, 0.0);

        zSpeed = Math.min(Math.max(zSpeed, -1.7), 1.7);
        xSpeed = Math.min(Math.max(xSpeed, -1.5), 1.5);
        angularSpeed = Math.min(Math.max(angularSpeed, -0.5), 0.5);
        
        swerveSubsystem.setRobotOrientedSpeed(-zSpeed, xSpeed, angularSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotOrientedSpeed(0.01, 0, 0);
        LEDConstants.turboEffect = true;

    }

    @Override
    public boolean isFinished() {
        return (xAlignPIDController.atSetpoint() && zAlignPIDController.atSetpoint() && angularAlignPIDController.atSetpoint()) || !(limelightSubsystem.getID(LimelightConstants.LimelightReefRight) > 0);
    }

}