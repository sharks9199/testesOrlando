package frc.robot;

import java.util.List;

// ======================= IMPORTAÇÃO DE BIBLIOTECAS =======================
    import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
    import com.pathplanner.lib.path.PathConstraints;
    import com.pathplanner.lib.path.PathPlannerPath;
    import com.pathplanner.lib.path.Waypoint;
    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.math.util.Units;
    import edu.wpi.first.wpilibj.Joystick;
    import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.button.JoystickButton;
    import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldPoses;
    import frc.robot.Constants.OIConstants;
    import frc.robot.Constants.elevatorConstants;
    import frc.robot.Constants.intakeConstants;
    import frc.robot.commands.SwerveJoystickCmd;
    import frc.robot.commands.Autos.AutoCollectCmd;
    import frc.robot.commands.Autos.AutoScoreCmd;
    import frc.robot.commands.Autos.Autos;
    import frc.robot.commands.Autos.CollectCmd;
    import frc.robot.commands.Elevator.ElevatorPidCmd;
    import frc.robot.commands.Intake.IntakePidCmd;
    import frc.robot.subsystems.LimelightSubsystem;
    import frc.robot.subsystems.IntakeSubsystem;
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
        // ============================================================================

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

        new JoystickButton(Joystick1, OIConstants.kCollectCoralButtonIdx).whileTrue(new AutoCollectCmd(swerveSubsystem));
        new JoystickButton(Joystick1, OIConstants.kScoreCoralButtonIdx).whileTrue(new AutoScoreCmd(swerveSubsystem));
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
        LoadAutonomousChoices();
    }
    // =======================================================

    // ========== EXECUTA QUANDO O TELEOPERADO INICIAR ==========
    public void doWhenTeleopInit() {
        elevatorConstants.elevatorSetpoint = elevatorSubsystem.getPosition();
        intakeConstants.intakeSetpoint = intakeSubsystem.getPosition();
    }
    // =======================================================
    
    // ========== TRAJECTORY FOLLOWER ==========
    public Command followTrajectory(Pose2d endPose) {
        Pose2d currentPose = swerveSubsystem.getPoseEstimator();

        System.out.println("Oi");

        // The rotation component in these poses represents the direction of travel
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        Pose2d endPos = new Pose2d(currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                new PathConstraints(
                        1.0, 1.0,
                        Units.degreesToRadians(360), Units.degreesToRadians(540)),
                null, // Ideal starting state can be null for on-the-fly paths
                new GoalEndState(0.0, currentPose.getRotation()));

        return AutoBuilder.followPath(path);
    }
    // =======================================================

    // ================== COMANDO AUTÔNOMO ==================
    // Adiciona as opções de Autônomo
    public void LoadAutonomousChoices() {
        // Define a opção padrão
        AutoChooser.setDefaultOption("Null", null);

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
