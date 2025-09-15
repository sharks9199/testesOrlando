package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.climbConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class Autos {
    public static Command L2Algae(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L2AlgaePosition)),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L2AlgaePosition))
            );
        }
        
    public static Command L3Algae(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L3AlgaePosition)),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(12))
            );
    }

    public static Command ChangeToLimelightCoral(){
        return Commands.sequence(
            new InstantCommand(()-> LimelightConstants.LimelightToUpdatePose = LimelightConstants.LimelightCoral)
            );
            
    }

    public static Command ChangeToLimelightReef(){
        return Commands.sequence(
            new InstantCommand(()-> LimelightConstants.LimelightToUpdatePose = LimelightConstants.LimelightReefRight)
            );
            
    }

    public static Command ChangeToLimelightReefLeft(){
        return Commands.sequence(
            new InstantCommand(()-> LimelightConstants.LimelightToUpdatePose = LimelightConstants.LimelightReefLeft)
            );
            
    }

    public static Command L1Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L1Position)),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L1Position))
            );
    }

    public static Command L2Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L2Position)),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L2Position))
            );
    }

    public static Command L3Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L3Position)),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L3Position))
            );
    }

    public static Command L4Position(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        return Commands.sequence(
            new WaitUntilCommand(()-> !intakeConstants.intakeCollecting),
            new InstantCommand(()-> intakeSubsystem.changeSetpoint(intakeConstants.L4Position)),
            new InstantCommand(()-> elevatorSubsystem.changeSetpoint(elevatorConstants.L4Position))
            );
    }
    
}
