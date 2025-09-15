package frc.robot.commands;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import edu.wpi.first.math.filter.SlewRateLimiter;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.math.kinematics.SwerveModuleState;
    import edu.wpi.first.wpilibj2.command.Command;
    import java.util.function.Supplier;
    import frc.robot.Constants.DriveConstants;
    import frc.robot.Constants.LEDConstants;
    import frc.robot.Constants.OIConstants;
    import frc.robot.Constants.elevatorConstants;
    import frc.robot.subsystems.ElevatorSubsystem;
    import frc.robot.subsystems.LEDSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class SwerveJoystickCmd extends Command {
    // =================== INSTANCIA OS SUBSISTEMAS E VARIAVEIS =================
    private final SwerveSubsystem swerveSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Double> xSpdFunctionTwo, ySpdFunctionTwo, turningSpdFunctionTwo;
    private final Supplier<Boolean> resetEncoderButton;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private double xSpeed, ySpeed, turningSpeed, rate;
    private ChassisSpeeds chassisSpeeds;
    // ============================================================================

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, LEDSubsystem ledSubsystem, ElevatorSubsystem elevatorSubsystem, Supplier<Double> xSpdFunction,
                Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Double> xSpdFunctionTwo,
                Supplier<Double> ySpdFunctionTwo, Supplier<Double> turningSpdFunctionTwo, Supplier<Boolean> resetEncoderButton) {
        
        this.swerveSubsystem = swerveSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.xSpdFunctionTwo = xSpdFunctionTwo;
        this.ySpdFunctionTwo = ySpdFunctionTwo;
        this.turningSpdFunctionTwo = turningSpdFunctionTwo;
        this.resetEncoderButton = resetEncoderButton;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (resetEncoderButton.get()){
            swerveSubsystem.resetSwerve();
        }
   
        // 1. Coleta a posição do Joystick
        xSpeed = xSpdFunction.get();
        ySpeed = ySpdFunction.get();
        turningSpeed = DriveConstants.ShowRoomMode ? 0.3 : turningSpdFunction.get();

        // 2. Aplica uma tolerância
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Otimiza a velocidade

        if (DriveConstants.lowSpeed){
            rate = DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        } else if (elevatorSubsystem.getPosition() >= elevatorConstants.L2Position - 5){
            rate = 1;
        } else {
            rate = 1;
        }
        
        xSpeed = xLimiter.calculate(xSpeed) * rate;
        ySpeed = yLimiter.calculate(ySpeed) * rate;
        turningSpeed = turningLimiter.calculate(turningSpeed) * rate;

        // 5. Calcula a velocidade do robô, em relação a arena
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());

        if (LEDConstants.turboEffect){
            ledSubsystem.setTurboPattern(Math.max(Math.max(Math.abs(xSpeed), Math.abs(ySpeed)), Math.abs(turningSpeed)));
        }

        // 6. Calcula a velocidade individual de cada módulo Swerve
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 7. Move os módulos para a posição e aceleração desejadas
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
