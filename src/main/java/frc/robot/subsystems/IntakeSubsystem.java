package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeMotor = new SparkMax(intakeConstants.intakeMotorID, MotorType.kBrushless);
    private final SparkMaxConfig intakeMotorConfig;

    public IntakeSubsystem (){
        intakeMotorConfig = new SparkMaxConfig();
        intakeMotorConfig.idleMode(IdleMode.kBrake);
        intakeMotorConfig.inverted(true);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);     
    }

    public double getPosition(){
        return intakeMotor.getEncoder().getPosition();
    }

    public void setStopMode(){
        intakeMotor.set(0);
    }

    public void setMotor(double speed){
        intakeMotor.set(speed);
    }

    public void changeSetpoint(double setpoint){
        intakeConstants.intakeSetpoint = setpoint;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake", getPosition());
    }

}
