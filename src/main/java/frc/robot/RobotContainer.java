package frc.robot;

import com.ctre.phoenix6.SignalLogger;
// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import com.pathplanner.lib.auto.AutoBuilder;
    import com.pathplanner.lib.auto.NamedCommands;
    import com.pathplanner.lib.commands.PathPlannerAuto;
    import com.pathplanner.lib.events.EventTrigger;
    import com.pathplanner.lib.path.PathPlannerPath;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.networktables.GenericEntry;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.button.JoystickButton;
    import edu.wpi.first.wpilibj2.command.button.POVButton;
    import frc.robot.Constants.DriveConstants;
    import frc.robot.Constants.FieldPoses;
    import frc.robot.Constants.LEDConstants;
    import frc.robot.Constants.LimelightConstants;
    import frc.robot.Constants.OIConstants;
    import frc.robot.Constants.climbConstants;
    import frc.robot.Constants.elevatorConstants;
    import frc.robot.Constants.intakeConstants;
    import frc.robot.commands.SwerveJoystickCmd;
    import frc.robot.commands.Autos.AutoAlignLeftCmd;
    import frc.robot.commands.Autos.AutoAlignRightCmd;
    import frc.robot.commands.Autos.Autos;
    import frc.robot.commands.Autos.CollectCmd;
    import frc.robot.commands.Autos.MoveForwardCmd;
    import frc.robot.commands.Climb.ClimbCmd;
    import frc.robot.commands.Elevator.ElevatorPidCmd;
    import frc.robot.commands.Intake.IntakePidCmd;
    import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShuffleManager;
import frc.robot.subsystems.IntakeSubsystem;
    import frc.robot.subsystems.LEDSubsystem;
    import frc.robot.subsystems.ClimbSubsystem;
    import frc.robot.subsystems.ElevatorSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
// ============================================================================

public class RobotContainer {
    //Cria uma instância do gestor de Tabs do ShuffleBoard
    public static ShuffleManager tabscontroller = new ShuffleManager();
    // Cria o objeto para a escolha do autônomo
    SendableChooser<Command> AutoChooser = AutoBuilder.buildAutoChooser();
    
    // ======================== INSTANCIA OS SUBSISTEMAS =========================
    private final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    private final static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final static LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
    private final static LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final static ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    // ============================================================================
    
    // ======================== INSTANCIA OS JOYSTICKS =========================
    public final static Joystick Joystick1 = new Joystick(OIConstants.kDriverControllerPort);
    private final static Joystick Joystick2 = new Joystick(OIConstants.kSecondDriverControllerPort);
    // ============================================================================
    
    // ======================== INSTANCIA O SHUFFLEBOARD =========================
    ShuffleboardTab intakeTab, limelightTab, climbTab, elevatorTab, configTab;
    public GenericEntry TXLSetpointEntry, TZLSetpointEntry, RYLSetpointEntry, TXRSetpointEntry, TZRSetpointEntry , RYRSetpointEntry;
    public GenericEntry TXLSpeedEntry, TZLSpeedEntry, RYLSpeedEntry, TXRSpeedEntry, TZRSpeedEntry, RYRSpeedEntry;
    public PathPlannerPath pathSecond, pathThird, pathFirstPosition, pathThirdPosition;

    // ShuffleBoardConstants shuffleBoardConstants = new ShuffleBoardConstants();
    // ============================================================================

    public RobotContainer() {
        
        
        try{
            // Load the path you want to follow using its name in the GUI
            // wasdwas dwasd wasd wasd wasd w

            pathSecond = PathPlannerPath.fromPathFile("Second");
            pathThird = PathPlannerPath.fromPathFile("Third");
            pathFirstPosition = PathPlannerPath.fromPathFile("FirstPosition");
            pathThirdPosition = PathPlannerPath.fromPathFile("ThirdPosition");

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }

        
        NamedCommands.registerCommand("FirstRed", swerveSubsystem.pathfindToStartingPoseRed(()-> pathFirstPosition));
        NamedCommands.registerCommand("SecondRed", swerveSubsystem.pathfindToStartingPoseRed(()-> pathThirdPosition));
        NamedCommands.registerCommand("FirstBlue", swerveSubsystem.pathfindToStartingPoseBlue(()-> pathFirstPosition));
        
        NamedCommands.registerCommand("AutoAlignRight", new AutoAlignRightCmd(swerveSubsystem, limelightSubsystem, ledSubsystem, climbSubsystem));
        NamedCommands.registerCommand("AutoAlignLeft", new AutoAlignLeftCmd(swerveSubsystem, limelightSubsystem, ledSubsystem, climbSubsystem));
        NamedCommands.registerCommand("MoveForward", new MoveForwardCmd(swerveSubsystem, limelightSubsystem, ledSubsystem, 0.2));
        NamedCommands.registerCommand("AutoCollectCmd", new CollectCmd(elevatorSubsystem, intakeSubsystem, ()-> false, intakeConstants.AutoCollectPosition));
        
        NamedCommands.registerCommand("StopIntake", new InstantCommand(()-> intakeSubsystem.setIntake(0)));
        NamedCommands.registerCommand("Shoot", new InstantCommand(()-> intakeSubsystem.setIntake(-0.5)));
        NamedCommands.registerCommand("IntakeBack", new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.UpPosition)));
        NamedCommands.registerCommand("IntakeCollect", new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L1Position)));
        NamedCommands.registerCommand("IntakeL4", new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L4AutoPosition)));
        NamedCommands.registerCommand("IntakeUp", new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.UpPosition)));
        
        NamedCommands.registerCommand("ElevatorL4", new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L4Position)));
        NamedCommands.registerCommand("ElevatorL1", new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L1Position)));
        
        NamedCommands.registerCommand("toSecond", swerveSubsystem.pathfindThenFollowPath(() -> pathSecond));
        NamedCommands.registerCommand("toThird", swerveSubsystem.pathfindThenFollowPath(() -> pathThird));
        
        NamedCommands.registerCommand("TurnHoodOn", new InstantCommand(()-> intakeSubsystem.setHood(-0.15)));
        NamedCommands.registerCommand("ClawUp", new InstantCommand(()-> climbConstants.clawSetpoint = 1.6));
        
        NamedCommands.registerCommand("LimeReef", Autos.ChangeToLimelightReef());
        NamedCommands.registerCommand("LimeReefLeft", Autos.ChangeToLimelightReefLeft());
        NamedCommands.registerCommand("LimeCoral", Autos.ChangeToLimelightCoral());

        NamedCommands.registerCommand("One", new InstantCommand(()-> LimelightHelpers.setPipelineIndex(LimelightConstants.LimelightToUpdatePose, 1)));
        NamedCommands.registerCommand("Zero", new InstantCommand(()-> LimelightHelpers.setPipelineIndex(LimelightConstants.LimelightToUpdatePose, 0)));
        NamedCommands.registerCommand("Three", new InstantCommand(()-> LimelightHelpers.setPipelineIndex(LimelightConstants.LimelightReefLeft, 3)));
        NamedCommands.registerCommand("Two", new InstantCommand(()-> LimelightHelpers.setPipelineIndex(LimelightConstants.LimelightReefLeft,2)));

        new EventTrigger("IntakeCollect").onTrue(new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L1Position)));
        new EventTrigger("IntakeUp").onTrue(new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.UpPosition)));
        new EventTrigger("IntakeL4").onTrue(new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L4AutoPosition)));
        new EventTrigger("Subir").onTrue(new InstantCommand(()-> System.out.println("SUBIU ==============================")));
        
        new EventTrigger("One").onTrue(new InstantCommand(()-> limelightSubsystem.setPipeline(1, LimelightConstants.LimelightReefRight)));
        new EventTrigger("ElevatorL4").onTrue(new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L4Position)));
        new EventTrigger("L1Position").onTrue(Autos.L1Position(elevatorSubsystem, intakeSubsystem));
        new EventTrigger("ClawUp").onTrue(new InstantCommand(()-> climbConstants.clawSetpoint = 1));
        new EventTrigger("CoralStation").onTrue(new InstantCommand(()-> LimelightConstants.LimelightToUpdatePose = LimelightConstants.LimelightCoral));
        
        new EventTrigger("CoralStation").onTrue(new InstantCommand(()-> LimelightConstants.LimelightToUpdatePose = LimelightConstants.LimelightCoral));

        // =========================== COMANDOS PADRÕES ===========================
        // Comando padrão swerve operado por joystick
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem, ledSubsystem, elevatorSubsystem, () -> -Joystick1.getY(), () -> -Joystick1.getX(),
                () -> -OIConstants.getGyroAxis(Joystick1), () -> -Joystick2.getY(), () -> -Joystick2.getX(),
                () -> -OIConstants.getGyroAxis(Joystick2),() -> Joystick1.getRawButton(OIConstants.kResetEncodersButtonIdx)));
        
        //Comando padrão para controle do Elevador
        elevatorSubsystem.setDefaultCommand(new ElevatorPidCmd(elevatorSubsystem, () -> Joystick2.getPOV()));
        
        //Comando padrão para controle do Intake
        intakeSubsystem.setDefaultCommand(new IntakePidCmd(intakeSubsystem, () -> Joystick2.getPOV(), 
                () -> Joystick2.getRawButton(OIConstants.kIntakeInputButtonIdx) , () -> Joystick2.getRawButton(OIConstants.kIntakeOutputButtonIdx), 
                () -> Joystick2.getRawButton(OIConstants.kHoodInputButtonIdx), () -> Joystick2.getRawButton(OIConstants.kHoodOutputButtonIdx)));
                
        //Comando padrão para controle do Climb
        climbSubsystem.setDefaultCommand(new ClimbCmd(climbSubsystem, () -> Joystick2.getRawButton
                (OIConstants.kClimbUpButtonIdx), () -> Joystick2.getRawButton(OIConstants.kClimbDownButtonIdx), 
                () -> Joystick1.getRawButton(OIConstants.kClawOpenButtonIdx), () -> Joystick1.getRawButton(OIConstants.kClawCloseButtonIdx)));

        AutoChooser.setDefaultOption("Null", null);

        AutoChooser.addOption("Middle2", new PathPlannerAuto("Middle"));
        AutoChooser.addOption("Position2", new PathPlannerAuto("Position"));
        AutoChooser.addOption("PositionBlue", new PathPlannerAuto("BluePosition"));

        // Coloca no ShuffleBoard a janela para escolher o Autônomo
        SmartDashboard.putData("Autonomo", AutoChooser);
        
        // Atribui as funções para cada botão do Controle
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // =========================== CONTROLE DO SWERVE ===========================
        // Acionar Coleta Automatizada 
        new JoystickButton(Joystick1, OIConstants.kCollectButtonIdx)
                .toggleOnTrue(new CollectCmd(elevatorSubsystem, intakeSubsystem, () -> Joystick1.getRawButtonPressed(OIConstants.kCollectButtonIdx), intakeConstants.CollectPosition));
        
        new JoystickButton(Joystick1, OIConstants.kAlignReefLeftButtonIdx)
                .whileTrue(new AutoAlignLeftCmd(swerveSubsystem, limelightSubsystem, ledSubsystem, climbSubsystem));
        new JoystickButton(Joystick1, OIConstants.kAlignReefRightButtonIdx)
                .whileTrue(new AutoAlignRightCmd(swerveSubsystem, limelightSubsystem, ledSubsystem, climbSubsystem));
        
        new JoystickButton(Joystick1, OIConstants.kShowRoomModeButtonIdx)
                .onTrue(new InstantCommand(() -> swerveSubsystem.changeShowRoomState()));

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

        //new JoystickButton(Joystick1, OIConstants.kClimbUpButtonIdx).onTrue(new InstantCommand(() -> climbSubsystem.changeClimbSetpoint(climbConstants.robotUpPosition)));
        new JoystickButton(Joystick1, OIConstants.kLowSpeedButtonIdx).onTrue(new InstantCommand(() -> DriveConstants.lowSpeed = !DriveConstants.lowSpeed));
        
        new JoystickButton(Joystick2, OIConstants.kRemoveAlgaeL3Idx).onTrue(Autos.L3Algae(elevatorSubsystem, intakeSubsystem));
        new JoystickButton(Joystick2, OIConstants.kRemoveAlgaeL2Idx).onTrue(Autos.L2Algae(elevatorSubsystem, intakeSubsystem));
        
        new JoystickButton(Joystick2, OIConstants.kIntakeL4OperatorButtonIdx).onTrue(new InstantCommand(() -> intakeSubsystem.changeSetpoint(intakeConstants.UpPosition)));
        new JoystickButton(Joystick2, OIConstants.kMoveForwardOperatorButtonIdx).whileTrue(new MoveForwardCmd(swerveSubsystem, limelightSubsystem, ledSubsystem, 0.2));
        
        new JoystickButton(Joystick2, OIConstants.kIntakeOutputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setIntake(0)));
        new JoystickButton(Joystick2, OIConstants.kIntakeInputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setIntake(0)));
        new JoystickButton(Joystick2, OIConstants.kHoodOutputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setHood(0)));
        new JoystickButton(Joystick2, OIConstants.kHoodInputButtonIdx).onFalse(new InstantCommand(() -> intakeSubsystem.setHood(0)));

        //new JoystickButton(Joystick1, OIConstants.kCollectCoralButtonIdx)
               //.whileTrue(swerveSubsystem.pathfindToPose(() -> getCollectPose()));
        // new JoystickButton(Joystick1, OIConstants.kCollectCoralButtonIdx)
        //        .onTrue(new InstantCommand(()-> limelightSubsystem.setPipeline(0, LimelightConstants.LimelightReefRight)));

        //  new JoystickButton(Joystick1, OIConstants.kCollectCoralLeftButtonIdx).whileTrue(AutoBuilder.pathfindToPose(getCollectPose(), AutoConstants.constraints));
        //  new JoystickButton(Joystick1, OIConstants.kCollectCoralButtonIdx).whileTrue(Autos.AutoAlignLeft(swerveSubsystem, limelightSubsystem, ledSubsystem, intakeSubsystem, climbSubsystem));
        //  new JoystickButton(Joystick1, OIConstants.kCollectCoralButtonIdx).whileTrue(Autos.AutoAlignRight(swerveSubsystem, limelightSubsystem, ledSubsystem, intakeSubsystem, climbSubsystem));

        //  new JoystickButton(Joystick1, OIConstants.kScoreCoralButtonIdx).whileTrue(new AutoScoreCmd(swerveSubsystem));
    };

    // ========== Retorna a Coral Station mais Próxima ==========
    public Pose2d getCollectPose() {
        // Obtemos a pose atual do robô
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();
        Translation2d currentTranslation = currentPose.getTranslation();

        // Calcula a distância do robô para as duas posições de referência
        double distanceToLeft = currentTranslation.getDistance(FieldPoses.kCoralLeft.getTranslation());
        double distanceToRight = currentTranslation.getDistance(FieldPoses.kCoralRight.getTranslation());

        // Verifica qual das posições (esquerda ou direita) está mais próxima e retorna a pose
        if (distanceToLeft < distanceToRight) {
            System.out.println("Left!");
            return FieldPoses.kCoralLeft;
        } else {
            System.out.println("Right!");
            return FieldPoses.kCoralRight;
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
        ledSubsystem.setPattern(LEDConstants.ledOff);
        swerveSubsystem.resetAllEncoders();

        SignalLogger.stop();
        
        limelightSubsystem.setPipeline(0, LimelightConstants.LimelightReefRight);
        limelightSubsystem.setPipeline(0, LimelightConstants.LimelightReefLeft);
    }
    // =======================================================

    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenAutoInit() {
        intakeSubsystem.changeSetpoint(intakeConstants.UpPosition);
        climbSubsystem.changeClawSetpoint(1.7);

        limelightSubsystem.setPipeline(0, LimelightConstants.LimelightReefRight);
        limelightSubsystem.setPipeline(0, LimelightConstants.LimelightReefLeft);
    }
    // =======================================================
    
    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenTeleopInit() {
        elevatorConstants.elevatorSetpoint = elevatorSubsystem.getPosition();
        intakeConstants.intakeSetpoint = intakeSubsystem.getPosition();
        climbConstants.climbSetpoint = climbSubsystem.getClimbPosition();
        climbConstants.clawSetpoint = climbSubsystem.getClawPosition();
        
        limelightSubsystem.setPipeline(2, LimelightConstants.LimelightReefRight);
        limelightSubsystem.setPipeline(2, LimelightConstants.LimelightReefLeft);

        LimelightConstants.LimelightToUpdatePose = LimelightConstants.LimelightReefRight;

    } 
    // =======================================================

    // Executa a opção escolhida de Autônomo
    public Command getAutonomousCommand() {
        return AutoChooser.getSelected();
    }
    // ========================================================

    
}
