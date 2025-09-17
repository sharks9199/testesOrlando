// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class ShuffleManager extends SubsystemBase {
  
  private GenericEntry kPSwerveModuleEntry;
  private GenericEntry kISwerveModuleEntry;
  private GenericEntry kDSwerveModuleEntry;
  private GenericEntry maxVelocityEntry;
  private GenericEntry maxAccelerationEntry;
  
  public ShuffleManager() 
  {
    kPSwerveModuleEntry = Shuffleboard.getTab("Atualizar PID das rodas")
      .add("kP",DriveConstants.kPSwerveModule)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min",0,"max",10))
      .getEntry();
    
    kISwerveModuleEntry = Shuffleboard.getTab("Atualizar PID das rodas")
      .add("kI",DriveConstants.kISwerveModule)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min",0,"max",0.1 ))
      .getEntry();

    kDSwerveModuleEntry = Shuffleboard.getTab("Atualizar PID das rodas")
      .add("kD",DriveConstants.kDSwerveModule)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min",0,"max",0.1))
      .getEntry();

    maxVelocityEntry = Shuffleboard.getTab("Atualizar PID das rodas")
      .add("Velocidade Máxima",DriveConstants.maxVelocity)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min",0,"max",1000))
      .getEntry();
      
      maxAccelerationEntry = Shuffleboard.getTab("Atualizar PID das rodas")
      .add("Aceleracao Máxima",DriveConstants.maxAcceleration)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min",0,"max",1000))
      .getEntry();

    
    Shuffleboard.getTab("Dados do Robo")
      .add("Tensao da bateria",RobotController.getBatteryVoltage())
      .withWidget(BuiltInWidgets.kVoltageView)
      .withProperties(Map.of("min",0,"max",13));
    Shuffleboard.getTab("Dados do Robo")
      .add("Temperatura da CPU",RobotController.getCPUTemp());

  }
// =================== GETTERS COM FALLBACK ===================

  public double getTurnKP() {
    return kPSwerveModuleEntry != null
      ? kPSwerveModuleEntry.getDouble(DriveConstants.kPSwerveModule)
      : DriveConstants.kPSwerveModule;
  }

  public double getTurnKI() {
    return kISwerveModuleEntry != null
      ? kISwerveModuleEntry.getDouble(DriveConstants.kISwerveModule)
      : DriveConstants.kISwerveModule;
  }

  public double getTurnKD() {
    return kDSwerveModuleEntry != null
      ? kDSwerveModuleEntry.getDouble(DriveConstants.kDSwerveModule)
      : DriveConstants.kDSwerveModule;
  }

  public double getTurnMaxVel() {
    return maxVelocityEntry != null
      ? maxVelocityEntry.getDouble(DriveConstants.maxVelocity)
      : DriveConstants.maxVelocity;
  }

  public double getTurnMaxAcc() {
    return maxAccelerationEntry != null
      ? maxAccelerationEntry.getDouble(DriveConstants.maxAcceleration)
      : DriveConstants.maxAcceleration;
  }

  // =============== APLICAR NO MÓDULO (chame periodicamente) ===============

  public void applyTurningPidTo(SwerveModule module) {
    if (module == null) return;
    module.turningPidController.setP(getTurnKP());
    module.turningPidController.setI(getTurnKI());
    module.turningPidController.setD(getTurnKD());
    module.turningPidController.setConstraints(
      new TrapezoidProfile.Constraints(getTurnMaxVel(), getTurnMaxAcc())
    );
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
