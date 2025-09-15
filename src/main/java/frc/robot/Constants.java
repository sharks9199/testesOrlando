package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

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

        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 7;

        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 4;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 8;

        public static final int kElevatorMotorPort = 8;

        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

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

        public static double kSpeed = 1;
        public static double kSpeedAngular = 1;
        public static boolean lowSpeed = true;
        public static boolean ShowRoomMode = false;

        public static double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
        public static double kTeleDriveMaxAngularSpeedRadiansPerSecond = // 
                kPhysicalMaxAngularSpeedRadiansPerSecond;
        public static double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static final double ksVolts = 0.29565;
        public static final double kvVoltSecondsPerMeter = 0.8224;
        public static final double kaVoltSecondsSquaredPerMeter = 1.4512;
        public static final double kPDriveVel = 1.1949;

    }

    public static final class FieldPoses {
        // Load the path you want to follow using its name in the GUI
        public static PathPlannerPath kCoralBlueLeftPath;
        public static PathPlannerPath kCoralBlueRightPath;

        public static final Pose2d kCoralBlueRight = new Pose2d(1.01, 0.98, new Rotation2d(0.96));
        public static final Pose2d kCoralBlueLeft = new Pose2d(0.95, 7.02, new Rotation2d(-1));
        public static final Pose2d kCoralRedRight = new Pose2d(16.61, 7.11, new Rotation2d(-2.2));
        public static final Pose2d kCoralRedLeft = new Pose2d(16.60, 1.05, new Rotation2d(2.1));
        
        public static final Optional<Alliance> alliance = DriverStation.getAlliance();
        
        public static final Pose2d kCoralRight = alliance.get() == Alliance.Blue ? kCoralBlueRight : kCoralRedRight;
        public static final Pose2d kCoralLeft = alliance.get() == Alliance.Blue ? kCoralBlueLeft : kCoralRedLeft;
        
        public static final Pose2d kGrid1 = new Pose2d(2.42, 3.89, new Rotation2d(0));
        public static final Pose2d kGrid2 = new Pose2d(3.23, 2.21, new Rotation2d(1));
        public static final Pose2d kGrid3 = new Pose2d(5.56, 2.07, new Rotation2d(2));
        public static final Pose2d kGrid4 = new Pose2d(6.32, 3.98, new Rotation2d(3.1));
        public static final Pose2d kGrid5 = new Pose2d(5.30, 5.68, new Rotation2d(-2.2));
        public static final Pose2d kGrid6 = new Pose2d(3.53, 5.57, new Rotation2d(-1));

        public static final Pose2d[] gridPoses = {kGrid1, kGrid2, kGrid3, kGrid4, kGrid5, kGrid6};
    }

    public static final class elevatorConstants {
        public static final int elevatorMotorID = 13;
        
        public static final double CollectPosition = 4.1;
        public static final double L1Position = 4.1;
        public static final double L2Position = 15.8;
        public static final double L3Position = 51.8;
        public static final double L4Position = 54;
        public static final double L2AlgaePosition = 49;
        public static final double L3AlgaePosition = 67;

        public static final double elevatorMax = 54;
        public static final double elevatorMin = 2;
        public static double elevatorSetpoint = 0;
    }

    public static final class LEDConstants {
        // =========================== LED OFF =================================
        public static final LEDPattern ledOff = LEDPattern.solid(Color.kBlack);
        // =======================================================================

        // =========================== Blinking Green ===========================
        public static LEDPattern baseGreen = LEDPattern.solid(Color.kGreen);
        public static LEDPattern baseBlack = LEDPattern.solid(Color.kBlack);
        // =======================================================================

        // =========================== Blinking Red ===========================
        public static LEDPattern baseRed = LEDPattern.solid(Color.kRed);
        public static LEDPattern blinkingRed = baseRed.blink(Seconds.of(0.1));
        // =======================================================================
        
        // =========================== Blinking Blue ============================
        public static LEDPattern baseBlue = LEDPattern.solid(Color.kBlue);
        public static LEDPattern blinkingBlue = baseBlue.blink(Seconds.of(0.1));
        // =======================================================================
        
        // =========================== Blinking Yellow ==========================
        public static LEDPattern baseYellow = LEDPattern.solid(Color.kYellow);
        public static LEDPattern blinkingYellow = baseYellow.blink(Seconds.of(0.1));
        // =======================================================================
        
        // ================================ Turbo ===============================
        public static double speed = 0;
        public static final LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kOrangeRed);
        public static LEDPattern mask = LEDPattern.progressMaskLayer(()-> speed);
        public static LEDPattern turboDisplay = base.mask(mask);
        // =======================================================================

        public static boolean turboEffect = true;
    }

    public static final class intakeConstants {
        public static final int intakeMotorID = 14;
        public static final int controlMotorID = 15;
        public static final int hoodMotorID = 16;
        public static final int CANrangeFirstID = 20;
        public static final int CANrangeSecondID = 21;

        public static final double kCANrangeDetectionLimit = 0.1;
        public static final double CollectPosition = 0.2;
        public static final double AutoCollectPosition = 0;
        public static final double BackPosition = 21;
        public static final double L1Position = 3.5;
        public static final double L2Position = 3.5;
        public static final double L3Position = 3.5;
        public static final double L4AutoPosition = 16;
        public static final double L4Position = 18;
        public static final double UpPosition = 24;

        public static final double L2AlgaePosition = 8.97;
        public static final double L3AlgaePosition = 10;

        public static final double intakeMaxSpeed = 0.4;
        public static final double intakeMax = 25;
        public static final double intakeMin = -0.1;
        public static Boolean intakeCollecting = false;
        public static double intakeSetpoint = 0;
    }

    public static final class climbConstants {
        public static final int clawMotorID = 17;
        public static final int planetaryMotorID = 18;

        public static final double openPosition = 0;
        public static final double closedPosition = 0;

        public static final double climbMaxSpeed = 0.5;
        public static final double climbMinSpeed = 0.5;
        public static final double climbMax = 1000;
        public static final double climbMin = -1000;

        public static final double robotUpPosition = 256;

        public static final double clawMax = 2.45;
        public static final double clawMin = 0.1;

        public static final double climbP = 0.7;
        public static final double clawP= 0.7;
        public static double climbSetpoint = 0;
        public static double clawSetpoint = 0;
    }

    public static final class AutoConstants {
        public static PathConstraints constraints = new PathConstraints(4.0, 3.5, 
        Units.degreesToRadians(360), Units.degreesToRadians(540));
        
        public static PathConstraints constraintsAuto = new PathConstraints(4.0, 3.5, 
        Units.degreesToRadians(360), Units.degreesToRadians(540));

        public static final double kPLimelightAlignX = 3;
        public static final double kPLimelightAlignZ = 3;
        public static final double kPLimelightAlignAngular = 0.1;

        public static final double kMaxZAcelleration = 0.5;
        public static final double kMaxXAcelleration = 3;
        public static final double kMaxAngularAcelleration = 3;

    }

    public static final class InterfaceConstants {
        public static boolean[][] fedReefPositions = new boolean[3][12];

    }
    
    public static final class LimelightConstants {
        public static String LimelightCoral = "limelight-coral"; 
        public static String LimelightReefLeft = "limelight-reefle";
        public static String LimelightReefRight = "limelight-reefri";
        public static String LimelightToUpdatePose = LimelightReefRight;
    }

    public static final class XboxConstants {
        public static final int kResetEncodersButtonIdx = 3;
        
    }

    public static final class LogitechConstants {
        public static final int kResetEncodersButtonIdx = 1;

    }

    public static final class OIConstants {
        public static Boolean isXbox = DriverStation.getJoystickIsXbox(0);

        public static double getGyroAxis(Joystick joystick){
            return isXbox ? joystick.getRawAxis(4) : joystick.getZ();
        }

        // ============================ BOTÕES DO CONTROLE ===========================
        //Controle Swerve
        public static final int kResetEncodersButtonIdx = isXbox ? XboxConstants.kResetEncodersButtonIdx : LogitechConstants.kResetEncodersButtonIdx;

        //Controle Garra
        public static final int kIntakeInputButtonIdx = 5;
        public static final int kIntakeOutputButtonIdx = 6;
        public static final int kHoodInputButtonIdx = 7;
        public static final int kHoodOutputButtonIdx = 8;
        public static final int kClimbUpButtonIdx = 9;
        public static final int kClimbDownButtonIdx = 10;
        public static final int kLowSpeedButtonIdx = 10;
        public static final int kClawOpenButtonIdx = 11;
        public static final int kClawCloseButtonIdx = 12;
        public static final int kIntakeL4OperatorButtonIdx = 2;
        public static final int kMoveForwardOperatorButtonIdx = 1;
        public static final int kRaiseIntakeButtonIdx = 90;
        public static final int kLowerIntakeButtonIdx = 270;
        
        public static final int kRaiseElevatorButtonIdx = 0;
        public static final int kLowerElevatorButtonIdx = 180;
        
        public static final int kAlignReefLeftButtonIdx = 5;
        public static final int kAlignReefRightButtonIdx = 6;
        public static final int kShowRoomModeButtonIdx = 3;
        public static final int kScoreCoralButtonIdx = 4;
        public static final int kCollectButtonIdx = 2;

        public static final int kRemoveAlgaeL2Idx = 4;
        public static final int kRemoveAlgaeL3Idx = 3;

        public static final int kL1ButtonIdx = 180;
        public static final int kL2ButtonIdx = 90;
        public static final int kL3ButtonIdx = 270;
        public static final int kL4ButtonIdx = 0;
        // ============================================================================

        public static final int kDriverControllerPort = 0;
        public static final int kSecondDriverControllerPort = 1;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 2;
        
        public static final double kDeadband = 0.05;  
        
        // ========================== Virtual Controller ===============================
        public static final int kThirdDriverControllerPort = 2;
        public static final int levelAdd = 1;
        public static final int levelRem = 2;
        public static final int l2Marker = 0;
        public static final int l3Marker = 1;
        public static final int l4Marker = 2;
        // ===========================================================================

    }

}

