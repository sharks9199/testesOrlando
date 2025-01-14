package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class AprilTagAllignCmd extends Command {

    private final LimelightSubsystem limelightSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    private PIDController xPidController;
    private PIDController zPidController;
    private PIDController angularPidController;

    private double xError, xSpeed, zError, zSpeed, angularError, angularSpeed;
    private int pipeline;

    public AprilTagAllignCmd(LimelightSubsystem limelightSubsystem, SwerveSubsystem swerveSubsystem, int pipeline) {
        this.limelightSubsystem = limelightSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.pipeline = pipeline;

        zPidController = new PIDController(2, 0, 0.00);
        zPidController.setTolerance(0.15);
        xPidController = new PIDController(5, 0, 0.00);
        xPidController.setTolerance(0.06);
        angularPidController = new PIDController(0.05, 0, 0.00);
        angularPidController.setTolerance(1);

        addRequirements(limelightSubsystem, swerveSubsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        limelightSubsystem.setPipeline(pipeline);
        
        xError = limelightSubsystem.getTX();
        zError = limelightSubsystem.getTZ();
        angularError = limelightSubsystem.getRY();

        System.out.println("Tx = " + xError);
        System.out.println("Tz = " + zError);
        System.out.println("Angular = " + angularError);

        xSpeed = xPidController.calculate(xError, 0);
        zSpeed = zPidController.calculate(zError, -1.5);
        angularSpeed = angularPidController.calculate(angularError, 0);
        
        xSpeed = Math.max(-5, Math.min(5, xSpeed));
        zSpeed = Math.max(-4.5, Math.min(4.5, zSpeed));
        angularSpeed = Math.max(-4, Math.min(4, angularSpeed));

        swerveSubsystem.setRobotOrientedSpeed(-zSpeed,-xSpeed,-angularSpeed);

        System.out.println("Alinhando");
    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("Alinhou");
        System.out.println("Tx = " + xError);
        System.out.println("Tz = " + zError);
        System.out.println("Angular = " + angularError);
        swerveSubsystem.stopModules();
    }


}
