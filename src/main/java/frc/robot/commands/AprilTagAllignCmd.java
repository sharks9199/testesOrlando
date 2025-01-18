package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        xPidController = new PIDController(0.1, 0, 0.00);
        xPidController.setTolerance(0.7);
        angularPidController = new PIDController(0.05, 0, 0.00);
        angularPidController.setTolerance(2);

        addRequirements(limelightSubsystem);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        limelightSubsystem.setPipeline(pipeline);
        
        angularError = limelightSubsystem.getRY();
        xError = angularError < 5 && angularError > -5 ? limelightSubsystem.getTX() : 0;
        zError = limelightSubsystem.getTZ();

        System.out.println("Tx = " + xPidController.atSetpoint());
        System.out.println("Tz = " + zPidController.atSetpoint());
        System.out.println("Angular = " + angularPidController.atSetpoint());

        zSpeed = zPidController.calculate(zError, 1.5);
        xSpeed = xPidController.calculate(xError, 0);
        angularSpeed = angularPidController.calculate(angularError, 0);
        
        int maxSpeed = 1;
        xSpeed = xPidController.atSetpoint() ? 0 : Math.max(-maxSpeed, Math.min(maxSpeed, xSpeed));
        zSpeed = zPidController.atSetpoint() ? 0 : Math.max(-maxSpeed, Math.min(maxSpeed, zSpeed));
        angularSpeed = angularPidController.atSetpoint() ? 0 : Math.max(-maxSpeed, Math.min(maxSpeed, angularSpeed));

        SmartDashboard.putBoolean("X PID", xPidController.atSetpoint());
        SmartDashboard.putBoolean("Z PID", zPidController.atSetpoint());
        SmartDashboard.putBoolean("Angular PID", angularPidController.atSetpoint());

        SmartDashboard.putNumber("X Speed", xSpeed);
        SmartDashboard.putNumber("Z Speed", zSpeed);
        SmartDashboard.putNumber("Angular Speed", angularSpeed);

        SmartDashboard.putNumber("X Error", xError);
        SmartDashboard.putNumber("Z Error", zError);
        SmartDashboard.putNumber("Angular Error", angularError);
        
        swerveSubsystem.setRobotOrientedSpeed(-zSpeed, -xSpeed, angularSpeed);

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
