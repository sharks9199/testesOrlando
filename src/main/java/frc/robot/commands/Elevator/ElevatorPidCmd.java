package frc.robot.commands.Elevator;

import java.util.function.Supplier;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPidCmd extends Command {
    private final Supplier<Integer> pov;
    private double error;

    ElevatorSubsystem elevatorSubsystem;
    ProfiledPIDController controller = new ProfiledPIDController(0.1, 0.0, 0.0,
    new TrapezoidProfile.Constraints(800, 600));
    
    public ElevatorPidCmd(ElevatorSubsystem elevatorSubsystem, Supplier<Integer> pov){
        this.elevatorSubsystem = elevatorSubsystem;
        this.pov = pov;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
        elevatorConstants.elevatorSetpoint = elevatorSubsystem.getPosition();
        controller.reset(elevatorSubsystem.getPosition());
    }

    @Override
    public void execute(){
        if(pov.get() == OIConstants.kRaiseElevatorButtonIdx){elevatorSubsystem.changeSetpoint(elevatorConstants.elevatorSetpoint + 0.65);} 
        if(pov.get() == OIConstants.kLowerElevatorButtonIdx){elevatorSubsystem.changeSetpoint(elevatorConstants.elevatorSetpoint - 0.65);}

        elevatorConstants.elevatorSetpoint = Math.min(Math.max(elevatorConstants.elevatorSetpoint, elevatorConstants.elevatorMin), elevatorConstants.elevatorMax);

        error = controller.calculate(elevatorSubsystem.getPosition(), elevatorConstants.elevatorSetpoint);

        //error = elevatorPidController.calculate(, );//Math.max(elevatorConstants.elevatorSetpoint, -198));
        elevatorSubsystem.setMotor(error);
    
    }

    @Override
    public void end(boolean interrupted){
    }

}
