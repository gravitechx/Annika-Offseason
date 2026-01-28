package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.COTSFalconSwerveConstants;

public final class Constants {
    public static final double periodicSpeed = Robot.kDefaultPeriod; // how quickly periodic methods are called.
                                                                     // (millis)

    public static final class Joystick {
        public static final double kStickDeadband = 0.12;
        public static final double kRotationDeadband = 0.2;
        public static final double kSlewRateLimit = 0.9; // larger the number, faster you can change output
        public static final int kPort = 0;
        public static final int kXAxis = 0;
        public static final int kYAxis = 1;
        public static final int kRotationAxis = 2;
    }
    // operator input

    public record SwerveConstants(int kDriveMotorID, int kAngleMotorID, int kCancoderID, double kAngleOffset) {
    }

    public static final class Swerve {
        public static final double kMaxSpeedMetersPerSec = 6.5;
        public static final double maxDriveSpeed = 0.4 * kMaxSpeedMetersPerSec;
        public static final double kMaxAngularVelocityRad = Math.PI;
        public static final COTSFalconSwerveConstants kSwerveSpecialtyModule = COTSFalconSwerveConstants
                .SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);
        public static final double kWheelCircumference = kSwerveSpecialtyModule.wheelCircumference;
        // distance from center of wheel to center of wheel (long side of car)
        public static final double kWheelBase = Units.inchesToMeters(24.75);
        // distance from middle of tire to middle of other tire (width of car)
        public static final double kTrackWidth = Units.inchesToMeters(24.75);
        // used to define kinematics (optional)
        public static final double halfWheelBase = kWheelBase / 2;
        public static final double halfTrackWidth = kTrackWidth / 2;
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
                new Translation2d(halfWheelBase, halfTrackWidth),
                new Translation2d(halfWheelBase, -halfTrackWidth),
                new Translation2d(-halfWheelBase, halfTrackWidth),
                new Translation2d(-halfWheelBase, -halfTrackWidth));

        public static final class FLModule {
            public static final int kDriveID = 3;
            public static final int kTurnID = 4;
            public static final int kEncoderID = 2;
            public static final double kOffsetAngle = -0.467773;
            public static final SwerveConstants kSwerveConstants = new SwerveConstants(kDriveID, kTurnID, kEncoderID,
                    kOffsetAngle);
        }

        public static final class FRModule {
            public static final int kDriveID = 1;
            public static final int kTurnID = 2;
            public static final int kEncoderID = 1;
            public static final double kOffsetAngle = 0.168945;
            public static final SwerveConstants kSwerveConstants = new SwerveConstants(kDriveID, kTurnID, kEncoderID,
                    kOffsetAngle);
        }

        public static final class BLModule {
            public static final int kDriveID = 7;
            public static final int kTurnID = 8;
            public static final int kEncoderID = 4;
            public static final double kOffsetAngle = -0.211182;
            public static final SwerveConstants kSwerveConstants = new SwerveConstants(kDriveID, kTurnID, kEncoderID,
                    kOffsetAngle);
        }

        public static final class BRModule {
            public static final int kDriveID = 5;
            public static final int kTurnID = 6;
            public static final int kEncoderID = 3;
            public static final double kOffsetAngle = 0.006836;
            public static final SwerveConstants kSwerveConstants = new SwerveConstants(kDriveID, kTurnID, kEncoderID,
                    kOffsetAngle);
        }

        public static final CTREConfigs kCTREConfigs = new CTREConfigs();

        public static final class CTRE {
            /* Module Gear Ratios */
            public static final double kDriveGearRatio = Swerve.kSwerveSpecialtyModule.driveGearRatio;
            public static final double kAngleGearRatio = Swerve.kSwerveSpecialtyModule.angleGearRatio;

            /* Motor Inverts */
            public static final InvertedValue kAngleMotorInvert = InvertedValue.Clockwise_Positive;
            public static final InvertedValue kDriveMotorInvert = InvertedValue.Clockwise_Positive;

            /* Angle Encoder Invert */
            public static final SensorDirectionValue kCanCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

            /* Swerve Current Limiting */
            public static final int kAngleCurrentLimit = 25;
            public static final int kAngleCurrentThreshold = 40;
            public static final double kAngleCurrentThresholdTime = 0.1;
            public static final boolean kAngleEnableCurrentLimit = true;

            public static final int kDriveCurrentLimit = 35;
            public static final int kDriveCurrentThreshold = 60;
            public static final double kDriveCurrentThresholdTime = 0.1;
            public static final boolean kDriveEnableCurrentLimit = true;

            /*
             * These values are used by the drive falcon to ramp in open loop and closed
             * loop driving.
             * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
             */
            public static final double kOpenLoopRamp = .25;
            public static final double kClosedLoopRamp = 0;

            /* Angle Motor PID Values */
            public static final double angleKP = Swerve.kSwerveSpecialtyModule.angleKP;
            public static final double angleKI = Swerve.kSwerveSpecialtyModule.angleKI;
            public static final double angleKD = Swerve.kSwerveSpecialtyModule.angleKD;
            public static final double angleKF = Swerve.kSwerveSpecialtyModule.angleKF;

            /* Drive Motor PID Values */
            public static final double driveKP = 1;
            public static final double driveKI = 0.0;
            public static final double driveKD = 0.0;
            public static final double driveKF = 0.0;

            /*
             * Drive Motor Characterization Values
             * Divide SYSID values by 12 to convert from volts to percent output for CTRE
             */
            public static final double driveKS = (0.32 / 12);
            public static final double driveKV = (1.51 / 12);
            public static final double driveKA = (0.27 / 12);
            /* Neutral Modes */
            public static final NeutralModeValue kAngleNeutralMode = NeutralModeValue.Coast;
            public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
        }

    }

    public static final class LimelightConstants {
        /*
         * x offset to branch can be found at
         * https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.
         * pdf (pg. 185 )
         * Source measurment: 6.38 (inches)
         * Hand measurment: 7.5 inches
         */
        public static final Translation3d translationToRobot = new Translation3d(Units.inchesToMeters(10.48),
                Units.inchesToMeters(0.582), Units.inchesToMeters(4.691)); // TODO fill this out later
        public static final Rotation3d rotationOffset = new Rotation3d(180, 20, 0);
        public static final Translation3d tagToBranchOffset = new Translation3d();
    }
}