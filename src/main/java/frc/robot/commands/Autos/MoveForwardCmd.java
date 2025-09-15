package frc.robot.commands.Autos;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.controller.PIDController;
    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.Constants.LEDConstants;
    import frc.robot.subsystems.LEDSubsystem;
    import frc.robot.subsystems.LimelightSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class MoveForwardCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private Timer timer;
    private PIDController forwardAlignPIDController;
    private double zSpeed, startFollowPosition, meters, modulePosition;
    // ============================================================================

    public MoveForwardCmd(SwerveSubsystem swerveSubsystem, LimelightSubsystem limelightSubsystem, 
                LEDSubsystem ledSubsystem, double meters) {
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.meters = meters;
        
        timer = new Timer();
        forwardAlignPIDController = new PIDController(4, 0, 0.00);
        forwardAlignPIDController.setTolerance(0.02);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() { 
        LEDConstants.turboEffect = false;

        timer.reset();
        timer.start();
        startFollowPosition = swerveSubsystem.getForwardPosition();
        ledSubsystem.setPattern(LEDConstants.baseRed);
    }

    @Override
    public void execute() {
        ledSubsystem.setPattern(LEDConstants.blinkingRed);

        modulePosition = swerveSubsystem.frontLeft.getModuleRotation2d().getDegrees();

        if (modulePosition > 90 || modulePosition < -90){
            zSpeed = -forwardAlignPIDController.calculate(swerveSubsystem.getForwardPosition(), startFollowPosition - meters);
            SmartDashboard.putNumber("Setpoint", startFollowPosition - meters);
        } else {
            zSpeed = forwardAlignPIDController.calculate(swerveSubsystem.getForwardPosition(), startFollowPosition + meters);
            SmartDashboard.putNumber("Setpoint", startFollowPosition + meters);
        }
        
        SmartDashboard.putNumber("Position", swerveSubsystem.getForwardPosition());

        zSpeed = Math.min(Math.max(zSpeed, -0.5), 0.5);
        swerveSubsystem.setRobotOrientedSpeed(zSpeed, 0, 0);

    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotOrientedSpeed(0, 0, 0);
        LEDConstants.turboEffect = true;
    }

    @Override
    public boolean isFinished() {
        return forwardAlignPIDController.atSetpoint() || 
        timer.get() >= 0.5;
    }

}