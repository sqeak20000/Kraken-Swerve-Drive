package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    /*
     * make all pistons IDs set to 40-59
     * all Motor IDs set to 10-29
     * all Sensor IDs set to 30-39
     */

    // Motors are all assumed to be TalonFX. TalonFX motors have an integrated
    // encoder which is why you don't see encoder ids
    // If Motors are diffents constants need to shifted accordingly

    // All values of -1 are place holders

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = .0977;
        // gear ratio is inversed (1/gear ratio) so we can multiply instead of dividing
        // the Rot to meter funtion
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        // 2048 is pulses per rotation of the motor
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters
                * (1.0 / 2048.0) * 0.9402;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI * (1.0 / 2048.0);

        // CTRE mesures their velcity in 100ms, so we multiply it by 10 to get 1s
        public static final double kDriveEncoderRPMS2MeterPerSec = kDriveEncoderRot2Meter * 10;
        public static final double kTurningEncoderRPMS2RadPerSec = kTurningEncoderRot2Rad * 10;
        // use guess and check to find. when the module is overshooting this needs to be
        // fine tuned
        public static final double kPTurning = 0.2;
    }

    public static final class DriveConstants {

        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(23);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(23);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 5;
        public static final int kBackRightDriveMotorPort = 3;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 7;
        public static final int kBackRightTurningMotorPort = 6;

        // Positive should be clockwise
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 4;
        public static final int kBackRightDriveAbsoluteEncoderPort = 1;

        // Positive should be clockwise
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;

        // To find set the motors forward record the value (don't inverse the value)
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 80.28;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 14.4; 
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 50.98; 
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 149.78; 

        // Max physical speed of our motors. Required for motor speed caculations
        // To find set the modules to 100% and see what speed cap out at
        public static final double kPhysicalMaxSpeedMetersPerSecond = 4.72;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 7 * 2 * Math.PI;

    }

    public static final class TeleDriveConstants {
        // Motion constants for joystick drive mode -> dependant on max speed
        // Max x/y speed of robot in this drive mode
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1; //used to be /4
        // Max rotation speed of robot in this drive mode
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 16;
        // Max x/y acceleration of robot in this drive mode
        public static final double kMaxAccelerationUnitsPerSecond = 8;
        // Max rotational acceleration of robot in this drive mode
        public static final double kMaxAngularAccelerationUnitsPerSecond = Math.PI * 2;
    }

    public static final class AutoConstants {
        // Motion constants for sequential path drive mode
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 14;
        public static final double kMaxAccelerationMetersPerSecondSquared = 5.5;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 2;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 1.5;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class TargetPosConstants {
        // Motion constants for target position drive mode
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeed = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 16;

        public static final double kForwardMaxAcceleration = 2;
        public static final double kBackwardMaxAcceleration = -12;
        public static final double kMaxAngularAcceleration = Math.PI / 3;
        public static final double kBackwardAngularAcceleration = -Math.PI * 9;

        public static final double kMinAngluarSpeedRadians = Math.PI / 16;
        public static final double kMinSpeedMetersPerSec = .2;

        public static final double kPDriveController = 1.9;
        public static final double kPAngleController = 1.9;
        public static final double kAcceptableDistanceError = 0.04;
        public static final double kAcceptableAngleError = 1.5;
    }

    public static final class OIConstants {
        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kXboxControllerPort = 2;

        // left joystick
        public static final int kLeftDriverYAxis = 1;
        public static final int kLeftDriverXAxis = 0;

        // right joystick
        public static final int kRightDriverRotAxis = 0;

        // area were joysticks will not activate
        public static final double kDeadband = 0.05;
    }

    public static final class SensorConstants {
        public static final int kPigeonID = 0;

        public static final double sizeOfFieldMeters = 16.53;
    }
    public static final class PneumaticsConstants {
        public static final int kCompressorPort = 1;
        public static final double kPulseDuration = .1; //in seconds
        public static final int kSolenoidOne = 2;
        public static final int kSolenoidTwo = 1;
        public static final int kSolenoidThree = 0;
    }
}
