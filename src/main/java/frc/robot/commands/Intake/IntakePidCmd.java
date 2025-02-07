package frc.robot.commands.Intake;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePidCmd extends Command {
    private final Supplier<Integer> pov;
    private double error;

    IntakeSubsystem intakeSubsystem;
    PIDController intakePidController = new PIDController(0.2,0,0);

    public IntakePidCmd(IntakeSubsystem intakeSubsystem, Supplier<Integer> pov){
        this.intakeSubsystem = intakeSubsystem;
        this.pov = pov;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if(pov.get() == 90){intakeSubsystem.changeSetpoint(intakeConstants.intakeSetpoint - 0.05);} 
        if(pov.get() == 270){intakeSubsystem.changeSetpoint(intakeConstants.intakeSetpoint + 0.05);}

        intakeConstants.intakeSetpoint = Math.min(Math.max(intakeConstants.intakeSetpoint, intakeConstants.intakeMin), intakeConstants.intakeMax);
        error = intakePidController.calculate(intakeSubsystem.getPosition(), intakeConstants.intakeSetpoint);
        
        SmartDashboard.putNumber("Intake Position", intakeSubsystem.getPosition());
        SmartDashboard.putNumber("Intake Speed", error);
        intakeSubsystem.setMotor(error);
    }

    @Override
    public void end(boolean interrupted){
    }

}
