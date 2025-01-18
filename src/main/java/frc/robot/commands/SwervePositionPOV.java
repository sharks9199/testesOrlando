package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwervePositionPOV extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Number> POVPressed;
    private double xSpeed;
    private double ySpeed;

    public SwervePositionPOV(SwerveSubsystem swerveSubsystem, double xSpeed, double ySpeed, Supplier<Number> POVPressed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.POVPressed = POVPressed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute() {
        System.out.println("POV angle = " + xSpeed);

        swerveSubsystem.setRobotSpeed(xSpeed, ySpeed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotSpeed(-(xSpeed/2), -(ySpeed/2), 0);

    }

    @Override
    public boolean isFinished() {
        return (POVPressed.get().doubleValue() == -1);

    }
}
