package frc.robot;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.commands.PathPlannerAuto;
    import com.pathplanner.lib.path.PathConstraints;
    import com.pathplanner.lib.path.PathPlannerPath;

    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.util.Units;
    import edu.wpi.first.wpilibj.Joystick;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.button.JoystickButton;
    import frc.robot.Constants.FieldPoses;
    import frc.robot.Constants.OIConstants;
    import frc.robot.commands.FollowPathCmd;
    import frc.robot.commands.SwerveJoystickCmd;
    import frc.robot.subsystems.LimelightSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class RobotContainer {
    // Cria o objeto para a escolha do autônomo
    SendableChooser<Command> AutoChooser = AutoBuilder.buildAutoChooser();

    // ======================== INSTÂNCIA OS SUBSISTEMAS =========================
    private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final static Joystick Joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    // ============================================================================

    public RobotContainer() {
        // =========================== COMANDOS PADRÕES ===========================
        // Comando padrão swerve operado por joystick
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem, limelightSubsystem, () -> -Joystick1.getY(), () -> -Joystick1.getX(),
                () -> -OIConstants.getGyroAxis(Joystick1), () -> false));
        // ============================================================================

        // Atribui as funções para cada botão do Controle
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // =========================== CONTROLE DO SWERVE ===========================
        // Resetar referência swerve
        new JoystickButton(Joystick1, OIConstants.kResetEncodersButtonIdx)
                .onTrue(new InstantCommand(() -> swerveSubsystem.resetSwerve()));
        // ============================================================================

        SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
                FieldPoses.kCoralBlueRight,
                new PathConstraints(
                        1.0, 1.0,
                        Units.degreesToRadians(360), Units.degreesToRadians(540)),
                0));

        new JoystickButton(Joystick1, 4).onTrue(new FollowPathCmd(swerveSubsystem));
    };

    // ========== EXECUTA QUANDO O ROBÔ INICIAR ==========
    public void doWhenRobotInit() {
        // Reseta a posição dos encoders
        swerveSubsystem.resetAllEncoders();
        LoadAutonomousChoices();
    }
    // =======================================================

    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenAutoInit() {
        swerveSubsystem.resetPoseEstimator(new Pose2d(0, 0, new Rotation2d()));
        LoadAutonomousChoices();
    }
    // =======================================================

    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenTeleopInit() {

    }
    // =======================================================

    // ========== TRAJECTORY FOLLOWER ==========
    public void followTrajectory(Pose2d endPose) {
        PathPlannerPath path = swerveSubsystem.createPath(endPose);
        AutoBuilder.followPath(path).schedule();
    }
    // =======================================================

    // ================== COMANDO AUTÔNOMO ==================
    // Adiciona as opções de Autônomo
    public void LoadAutonomousChoices() {
        // Define a opção padrão
        AutoChooser.setDefaultOption("Null", null);
        AutoChooser.addOption("Teste", new PathPlannerAuto("Auto"));

        // Coloca no ShuffleBoard a janela para escolher o Autônomo
        SmartDashboard.putData("Autonomo", AutoChooser);
    }

    // Executa a opção escolhida de Autônomo
    public Command getAutonomousCommand() {
        return AutoChooser.getSelected();
    }
    // ========================================================

}
