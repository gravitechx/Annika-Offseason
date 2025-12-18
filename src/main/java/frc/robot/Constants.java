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

public final class Constants {
    public static double armPOS = 0;
    public static int elevatorLevel = -3;
    public static double desiredHeight = 0;


    public static final double periodicSpeed = Robot.kDefaultPeriod; // how quickly periodic methods are called. (millis)
    public static final class Joystick{
        public static final double kStickDeadband = 0.12;
        public static final double kRotationDeadband = 0.2;
        public static final double kSlewRateLimit = 0.9; // larger the number, faster you can change output
        public static final int kPort = 0;
        public static final int kXAxis = 0;
        public static final int kYAxis = 1;
        public static final int kRotationAxis = 2;
    }
    //operator input

    public static final class LimelightConstants{
        /* x offset to branch can be found at https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings.pdf (pg. 185 )
            Source measurment: 6.38 (inches)
            Hand measurment: 7.5 inches
         */
        public static final Translation3d translationToRobot = new Translation3d(Units.inchesToMeters(10.48), Units.inchesToMeters(0.582), Units.inchesToMeters(4.691)); //TODO fill this out later
        public static final Rotation3d rotationOffset = new Rotation3d(180, 20, 0);
        public static final Translation3d tagToBranchOffset =  new Translation3d();
    }
}