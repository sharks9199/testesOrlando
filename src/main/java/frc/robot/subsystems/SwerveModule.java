package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFXConfiguration driveMotorConfig;
    private final SparkMax turningMotor;
    private final SparkMaxConfig turningMotorConfig;

    private final CANcoder absoluteEncoder;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    private final SparkClosedLoopController m_turningClosedLoopController;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        
        driveMotorConfig = new TalonFXConfiguration();

        driveMotorConfig.Feedback.withSensorToMechanismRatio(0.9);

        driveMotor.getConfigurator().apply(driveMotorConfig);

        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        turningMotorConfig = new SparkMaxConfig();
        m_turningClosedLoopController = turningMotor.getClosedLoopController();

        turningMotorConfig.inverted(turningMotorReversed);
        turningMotorConfig.idleMode(IdleMode.kCoast);

        turningMotorConfig.closedLoop.positionWrappingEnabled(true);
        turningMotorConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);
        turningMotorConfig.closedLoop.p(0.7);
        //turningMotorConfig.closedLoopRampRate(1);

        turningMotorConfig.encoder.positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningMotorConfig.encoder.velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningMotor.configure(turningMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setTolerance(0.0);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * ModuleConstants.kDriveEncoderRot2Meter;
    }

    public double getTurningPosition() {
        return turningMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getTurningVelocity() {
        return turningMotor.getEncoder().getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue().in(Radians);
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public Rotation2d getModuleRotation2d() {
        Rotation2d angleRotation2d = new Rotation2d(getAbsoluteEncoderRad());
        return angleRotation2d;
    }

    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad());
        System.out.println("Encoders Resetados");
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state, Rotation2d rotation2d) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        
        state.optimize(rotation2d);

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        
        SmartDashboard.putNumber("Turning", getTurningPosition());
        SmartDashboard.putNumber("Angle", state.angle.getRadians());
        
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

}
