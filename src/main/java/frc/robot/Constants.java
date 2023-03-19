package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;

public final class Constants {
    public static final class TrajectoryConstants{
        public static final double ksVolts = 1.4;
        public static final double kvVoltSecondsPerMeter = 3.4; 
        public static final double kaVoltSecondsSquaredPerMeter = 0.1;
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kPDriveVel = 1.88;
        public static final double kTrackwidthMeters = 0.63;
        public static final double kMaxAutoVoltage = 10;
        public static final boolean kGyroReversed = false;
        public static final double kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1524;
        static Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
static Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
static Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
static Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

// Creating my kinematics object using the wheel locations.
public final static   MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);
     //   public static final MecanumDriveKinematics kDriveKinematics = new MecanumDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    }
    public static final class JoystickConstants{
        // Port
        public static final int driverPort = 0;
       // public static final int coPort = 1;
    }
    public static final class DriveTrainConstants{
        // Port Constants

        // Invert Constants
        public static final InvertType leftMasterInvert = InvertType.None;
        public static final InvertType leftSlaveInvert = InvertType.None;
        public static final InvertType rightMasterInvert = InvertType.None;
        public static final InvertType rightSlaveInvert = InvertType.None;
        // Brake Constants
        public static final NeutralMode leftMasterBrake = NeutralMode.Brake;
        public static final NeutralMode leftSlaveBrake = NeutralMode.Brake;
        public static final NeutralMode rightMasterBrake = NeutralMode.Brake;
        public static final NeutralMode rightSlaveBrake = NeutralMode.Brake;
        // Safety Constants
        public static final boolean leftMasterSafety = false;
        public static final boolean leftSlaveSafety = false;
        public static final boolean rightMasterSafety = false;
        public static final boolean rightSlaveSafety = false;
        public static final boolean driveSafety = false;
        // Speed Constants
        public static final double driveLowSpeed = 0.35;
        public static final double driveDefaultSpeed = 0.6;
        public static final double driveHighSpeed = 1;
    }
   
    
   
    
}