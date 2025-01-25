package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;

public class Constants {
    public static final class ModuleConstants{
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.75);
        public static final double kDriveMotorGearRatio = 1.0 / 6.75;
        public static final double kTurningMotorGearRatio = 1.0 / (150.0/7.0);
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.3;
    }

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.56;
        // Distance between right and left wheels
        public static final double kWheelBase = 0.56;
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
                // Front Left 
                // Front Right
                // Back Left 
                // Back Right

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 11;

        public static final int kFrontLeftTurningMotorPort = 3;
        public static final int kBackLeftTurningMotorPort = 9;
        public static final int kFrontRightTurningMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 12;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 7;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 4;
        public static final int kBackRightDriveAbsoluteEncoderPort = 10;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kNoteCollectMaxSpeed = 2.5;

        public static double kSpeed = 1.5;
        public static double kSpeedAngular = 1.5;

        public static double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / kSpeed;
        public static double kTeleDriveMaxAngularSpeedRadiansPerSecond = // 
                kPhysicalMaxAngularSpeedRadiansPerSecond / kSpeedAngular;
        public static double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

        public static final double ksVolts = 0.29565;
        public static final double kvVoltSecondsPerMeter = 0.8224;
        public static final double kaVoltSecondsSquaredPerMeter = 1.4512;
        public static final double kPDriveVel = 1.1949;
    }

    public static final class FieldPoses {
        public static final Pose2d kCoralBlueRight = new Pose2d(1.28, 1.38, new Rotation2d(180));
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.0;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4.0;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 6.0;
        public static final double kSpeedLimiter = 4;

        public static final double kChargeStationXSpeed = 1;
        public static final double kPChargeStation = 0.04;

        public static final double kPLimelightAlignY = 0.05;
        public static final double kPLimelightAlignAngular = 0.1;

        public static final double kPOVSpeed = 1.5;
        public static final int kPOVForwardAngle = 0;
        public static final int kPOVBackwardAngle = 180;
        public static final int kPOVRightAngle = 270;
        public static final int kPOVLeftAngle = 90;
        public static final double kPMovePOV = 0.0000001;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }
    
    public static final class LimelightConstants {
        public static String LimelightID = "limelight-main"; 
    }

    public static final class XboxConstants {
        public static final int kResetEncodersButtonIdx = 3;
        
        public static final int kResetSpeakerBlueIdx = 7;
        public static final int kResetSpeakerRedIdx = 8;

        public static final int kCollectNoteIntakeIdx = 5;
        public static final int kShootNoteIntakeIdx = 6;

        public static final int kRaiseShooterIdx = 9;
        public static final int kLowerShoterIdx = 10;
    }

    public static final class LogitechConstants {
        public static final int kResetEncodersButtonIdx = 1;

        public static final int kResetSpeakerBlueIdx = 9;
        public static final int kResetSpeakerRedIdx = 10;

        public static final int kCollectNoteIntakeIdx = 5;
        public static final int kShootNoteIntakeIdx = 6;

        public static final int kRaiseShooterIdx = 11;
        public static final int kLowerShoterIdx = 12;
    }

    public static final class OIConstants {
        public static Boolean isXbox = DriverStation.getJoystickIsXbox(0);

        public static double getGyroAxis(Joystick joystick){
            double gyro = isXbox ? joystick.getRawAxis(4) : joystick.getZ();
            return gyro;
        }

        // ============================ BOTÕES DO CONTROLE ===========================
        //Controle Swerve
        public static final int kResetEncodersButtonIdx = isXbox ? XboxConstants.kResetEncodersButtonIdx : LogitechConstants.kResetEncodersButtonIdx;
        public static final int kResetSpeakerBlueIdx = isXbox ? XboxConstants.kResetSpeakerBlueIdx : LogitechConstants.kResetSpeakerBlueIdx;
        public static final int kResetSpeakerRedIdx = isXbox ? XboxConstants.kResetSpeakerRedIdx : LogitechConstants.kResetSpeakerRedIdx;
        
        public static final int kCollectNoteIntakeIdx = isXbox ? XboxConstants.kCollectNoteIntakeIdx : LogitechConstants.kCollectNoteIntakeIdx;
        public static final int kShootNoteIntakeIdx = isXbox ? XboxConstants.kShootNoteIntakeIdx : LogitechConstants.kShootNoteIntakeIdx;

        //Controle Garra
        public static final int kCollectNotePresetButtonIdx = 2;
        public static final int kShootNotePresetButtonIdx = 3;
        public static final int kIntakeInputButtonIdx = 5;
        public static final int kIntakeOutputButtonIdx = 6;

        public static final int kRaiseShooterIdx = isXbox ? XboxConstants.kRaiseShooterIdx : LogitechConstants.kRaiseShooterIdx;
        public static final int kLowerShoterIdx = isXbox ? XboxConstants.kLowerShoterIdx : LogitechConstants.kLowerShoterIdx;
        // ============================================================================

        public static final int kDriverControllerPort = 0;
        public static final int kSecondDriverControllerPort = 1;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 2;

        public static final int kSpeakerAlignButtonIdx = 7;
        public static final int kAMPAlignButtonIdx = 8;
        public static final int kSourceAlignButtonIdx = 5;
        
        public static final double kDeadband = 0.05;   

    }

}

