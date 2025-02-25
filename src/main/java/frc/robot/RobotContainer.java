package frc.robot;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.auto.NamedCommands;
    import com.pathplanner.lib.commands.PathPlannerAuto;
    import com.pathplanner.lib.events.EventTrigger;
    import com.pathplanner.lib.path.PathPlannerPath;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.Joystick;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.button.JoystickButton;
    import edu.wpi.first.wpilibj2.command.button.POVButton;
    import frc.robot.Constants.AutoConstants;
    import frc.robot.Constants.FieldPoses;
    import frc.robot.Constants.OIConstants;
    import frc.robot.Constants.climbConstants;
    import frc.robot.Constants.elevatorConstants;
    import frc.robot.Constants.intakeConstants;
    import frc.robot.commands.SwerveJoystickCmd;
    import frc.robot.commands.Autos.AutoAlignLeftCmd;
    import frc.robot.commands.Autos.AutoAlignRightCmd;
    import frc.robot.commands.Autos.AutoScoreCmd;
    import frc.robot.commands.Autos.Autos;
    import frc.robot.commands.Autos.CollectCmd;
    import frc.robot.commands.Climb.ClimbCmd;
    import frc.robot.commands.Elevator.ElevatorPidCmd;
    import frc.robot.commands.Intake.IntakePidCmd;
    import frc.robot.commands.LedStrip.LEDControlCmd;
    import frc.robot.subsystems.LimelightSubsystem;
    import frc.robot.subsystems.IntakeSubsystem;
    import frc.robot.subsystems.LedStripSubsystem;
    import frc.robot.subsystems.ClimbSubsystem;
    import frc.robot.subsystems.ElevatorSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class RobotContainer {
    // Cria o objeto para a escolha do autônomo
    SendableChooser<Command> AutoChooser = AutoBuilder.buildAutoChooser();
    
    // ======================== INSTANCIA OS SUBSISTEMAS =========================
    private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final static LedStripSubsystem ledSubsystem = new LedStripSubsystem();
    private final static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    public final static Joystick Joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    private final static Joystick Joystick2 = new Joystick(OIConstants.kSecondDriverControllerPort);
    // ============================================================================

    public RobotContainer() {
        // =========================== COMANDOS PADRÕES ===========================
        // Comando padrão swerve operado por joystick
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem, limelightSubsystem, () -> -Joystick1.getY(), () -> -Joystick1.getX(),
                () -> -OIConstants.getGyroAxis(Joystick1), () -> Joystick1.getRawButton(OIConstants.kResetEncodersButtonIdx)));
        
        //Comando padrão para controle do Elevador
        elevatorSubsystem.setDefaultCommand(new ElevatorPidCmd(elevatorSubsystem, () -> Joystick2.getPOV()));
        
        //Comando padrão para controle do Intake
        intakeSubsystem.setDefaultCommand(new IntakePidCmd(intakeSubsystem, () -> Joystick2.getPOV(), 
                () -> Joystick2.getRawButton(OIConstants.kIntakeInputButtonIdx) , () -> Joystick2.getRawButton(OIConstants.kIntakeOutputButtonIdx), 
                () -> Joystick2.getRawButton(OIConstants.kHoodInputButtonIdx), () -> Joystick2.getRawButton(OIConstants.kHoodOutputButtonIdx)));
                
        //Comando padrão para controle do Climb
        climbSubsystem.setDefaultCommand(new ClimbCmd(climbSubsystem, () -> Joystick1.getRawButton
                (OIConstants.kClimbUpButtonIdx), () -> Joystick1.getRawButton(OIConstants.kClimbDownButtonIdx), 
                () -> Joystick1.getRawButton(OIConstants.kClawOpenButtonIdx), () -> Joystick1.getRawButton(OIConstants.kClawCloseButtonIdx)));

        NamedCommands.registerCommand("Shoot", Autos.Shoot(intakeSubsystem));

        new EventTrigger("L3Position").onTrue(Autos.L3Position(elevatorSubsystem, intakeSubsystem));
        new EventTrigger("L4Position").onTrue(Autos.L4Position(elevatorSubsystem, intakeSubsystem));
        
        // Atribui as funções para cada botão do Controle
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // =========================== CONTROLE DO SWERVE ===========================

        // Acionar Coleta Automatizada 
        new JoystickButton(Joystick1, OIConstants.kCollectButtonIdx)
                .toggleOnTrue(new CollectCmd(elevatorSubsystem, intakeSubsystem, () -> Joystick1.getRawButtonPressed(OIConstants.kCollectButtonIdx)));
        
        new JoystickButton(Joystick1, OIConstants.kAlignReefLeftButtonIdx)
                .whileTrue(new AutoAlignLeftCmd(swerveSubsystem, limelightSubsystem));
        new JoystickButton(Joystick1, OIConstants.kAlignReefRightButtonIdx)
                .whileTrue(new AutoAlignRightCmd(swerveSubsystem, limelightSubsystem));

        // Presets para cada posição 
        new POVButton(Joystick1, OIConstants.kL1ButtonIdx, 0)
                .onTrue(Autos.L1Position(elevatorSubsystem, intakeSubsystem));
        new POVButton(Joystick1,OIConstants.kL2ButtonIdx, 0)
                .onTrue(Autos.L2Position(elevatorSubsystem, intakeSubsystem));
        new POVButton(Joystick1,OIConstants.kL3ButtonIdx, 0)
                .onTrue(Autos.L3Position(elevatorSubsystem, intakeSubsystem));
        new POVButton(Joystick1,OIConstants.kL4ButtonIdx, 0)
                .onTrue(Autos.L4Position(elevatorSubsystem, intakeSubsystem));
        
        // ============================================================================

        new JoystickButton(Joystick2, OIConstants.kIntakeOutputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setIntake(0)));
        new JoystickButton(Joystick2, OIConstants.kIntakeInputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setIntake(0)));
        new JoystickButton(Joystick2, OIConstants.kHoodOutputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setHood(0)));
        new JoystickButton(Joystick2, OIConstants.kHoodInputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setHood(0)));

        new JoystickButton(Joystick1, OIConstants.kCollectCoralButtonIdx).whileTrue(AutoBuilder.pathfindToPose(FieldPoses.kCoralBlueLeft, AutoConstants.constraints));
        new JoystickButton(Joystick1, OIConstants.kScoreCoralButtonIdx).whileTrue(new AutoScoreCmd(swerveSubsystem));
        new JoystickButton(Joystick1, OIConstants.kHoodInputButtonIdx).whileTrue(new LEDControlCmd(ledSubsystem));

    };

    // ========== Retorna a Coral Station mais Próxima ==========
    public Pose2d getCollectPose() {
        // Obtemos a pose atual do robô
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();
        Translation2d currentTranslation = currentPose.getTranslation();

        // Calcula a distância do robô para as duas posições de referência
        double distanceToLeft = currentTranslation.getDistance(FieldPoses.kCoralBlueLeft.getTranslation());
        double distanceToRight = currentTranslation.getDistance(FieldPoses.kCoralBlueRight.getTranslation());

        // Verifica qual das posições (esquerda ou direita) está mais próxima e retorna a pose
        if (distanceToLeft > distanceToRight) {
            System.out.println("Left!");
            return FieldPoses.kCoralBlueLeft;
        } else {
            System.out.println("Right!");
            return FieldPoses.kCoralBlueRight;
        }}
    // ==========================================================

    // ========== Retorna a Coral Station mais Próxima ==========
    public Pose2d getScorePose() {
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();
        Translation2d currentTranslation = currentPose.getTranslation();
        
        // Inicializa as variáveis de comparação
        Pose2d[] gridPoses = FieldPoses.gridPoses;
        Pose2d closestPose = null;
        double minDistance = Double.MAX_VALUE;
        double distance;

        // Itera sobre as poses e calcula a distância uma vez
        for (Pose2d pose : gridPoses) {
            distance = currentTranslation.getDistance(pose.getTranslation()); // Calcula a distância

            if (distance < minDistance) { // Atualiza se encontrar uma pose mais próxima
                minDistance = distance;
                closestPose = pose;
            }
        }

        return closestPose;
    }
    // ==========================================================

    // ========== EXECUTA QUANDO O ROBÔ INICIAR ==========
    public void doWhenRobotInit() {
        // Reseta a posição dos encoders
        swerveSubsystem.resetAllEncoders();
        LoadAutonomousChoices();
    }
    // =======================================================

    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenAutoInit() {
        LoadAutonomousChoices();
    }
    // =======================================================

    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenTeleopInit() {
        elevatorConstants.elevatorSetpoint = elevatorSubsystem.getPosition();
        intakeConstants.intakeSetpoint = intakeSubsystem.getPosition();
        climbConstants.climbSetpoint = climbSubsystem.getClimbPosition();
        climbConstants.clawSetpoint = climbSubsystem.getClawPosition();
    }
    // =======================================================

    // ================== COMANDO AUTÔNOMO ==================
    // Adiciona as opções de Autônomo
    public void LoadAutonomousChoices() {
        // Define a opção padrão
        AutoChooser.setDefaultOption("Null", null);

        try{
            FieldPoses.kCoralBlueLeftPath = PathPlannerPath.fromPathFile("Collect BL");
            FieldPoses.kCoralBlueRightPath = PathPlannerPath.fromPathFile("Collect BR");

        } catch (Exception e) {
            DriverStation.reportError("Files not Loaded: " + e.getMessage(), e.getStackTrace());
        }

        PathPlannerAuto autoBL = new PathPlannerAuto("Auto BL");
        AutoChooser.addOption("Teste", autoBL.beforeStarting(AutoBuilder.pathfindToPose(autoBL.getStartingPose(), AutoConstants.constraints)));

        // Coloca no ShuffleBoard a janela para escolher o Autônomo
        SmartDashboard.putData("Autonomo", AutoChooser);
    }

    // Executa a opção escolhida de Autônomo
    public Command getAutonomousCommand() {
        return AutoChooser.getSelected();
    }
    // ========================================================

}
