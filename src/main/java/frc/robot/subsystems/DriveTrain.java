// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.TrajectoryConstants;

public class DriveTrain extends SubsystemBase {
	enum SpeedMode{
		Low,
		Default,
		High
	}
	//private SpeedMode speedMode = SpeedMode.Default;

	private WPI_TalonSRX leftMaster;
	private WPI_TalonSRX leftSlave;
	private WPI_TalonSRX rightMaster;
	private WPI_TalonSRX rightSlave;

	private MecanumDrive m_drive;
  private Joystick m_stick= new Joystick(JoystickConstants.driverPort);
	public final MecanumDriveOdometry m_odometry;
    public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	private final Field2d m_field = new Field2d();
	public DriveTrain() {

		leftMaster = new WPI_TalonSRX(Constants.DriveTrainConstants.leftMasterPort);
		leftSlave = new WPI_TalonSRX(Constants.DriveTrainConstants.leftSlavePort);
		rightMaster = new WPI_TalonSRX(Constants.DriveTrainConstants.rightMasterPort);
		rightSlave = new WPI_TalonSRX(Constants.DriveTrainConstants.rightSlavePort);



		m_drive = new MecanumDrive(leftMaster, leftSlave,rightMaster,rightSlave);
		leftMaster.setInverted(Constants.DriveTrainConstants.leftMasterInvert);
		leftSlave.setInverted(Constants.DriveTrainConstants.leftSlaveInvert);
		rightMaster.setInverted(Constants.DriveTrainConstants.rightMasterInvert);
		rightSlave.setInverted(Constants.DriveTrainConstants.rightSlaveInvert);
		leftMaster.setNeutralMode(Constants.DriveTrainConstants.leftMasterBrake);
		leftSlave.setNeutralMode(Constants.DriveTrainConstants.leftSlaveBrake);
		rightMaster.setNeutralMode(Constants.DriveTrainConstants.rightMasterBrake);
		rightSlave.setNeutralMode(Constants.DriveTrainConstants.rightSlaveBrake);
		leftMaster.setSafetyEnabled(Constants.DriveTrainConstants.leftMasterSafety);
		leftSlave.setSafetyEnabled(Constants.DriveTrainConstants.leftSlaveSafety);
		rightMaster.setSafetyEnabled(Constants.DriveTrainConstants.rightMasterSafety);
		rightSlave.setSafetyEnabled(Constants.DriveTrainConstants.rightSlaveSafety);
		drive.setSafetyEnabled(Constants.DriveTrainConstants.driveSafety);
		leftMaster.setSensorPhase(true);
		rightMaster.setSensorPhase(true);

		m_odometry = new MecanumDriveOdometry(TrajectoryConstants.kDriveKinematics, m_gyro.getRotation2d(), new MecanumDriveWheelPositions());
        SmartDashboard.putData("Field", m_field);
		zeroHeading();
        resetEncoders();
	}


	@Override
	public void periodic() {
		
	}
//	public Pose2d getPose() {
//		return m_odometry.getPoseMeters();
	//}
	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360) * (TrajectoryConstants.kGyroReversed ? -1.0 : 1.0);
    }
	public void zeroHeading() {
        m_gyro.reset();
    }
    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      2.0, 2.0, Math.PI / 2.0, Rotation2d.fromDegrees(45.0));

    // Convert to wheel speeds
   // MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(getLeftMasterEncoderDistance(), getRightMasterEncoderDistance(), getLeftSlaveEncoderDistance(), getRightSlaveEncoderDistance());
    MecanumDriveWheelSpeeds wheelSpeeds = TrajectoryConstants.kDriveKinematics.toWheelSpeeds(speeds);
    // Get the individual wheel speeds
    double frontLeft = wheelSpeeds.frontLeftMetersPerSecond;
    double frontRight = wheelSpeeds.frontRightMetersPerSecond;
    double backLeft = wheelSpeeds.rearLeftMetersPerSecond;
    double backRight =wheelSpeeds.rearRightMetersPerSecond; 

    ChassisSpeeds chassisSpeeds = TrajectoryConstants.kDriveKinematics.toChassisSpeeds(wheelSpeeds);

// Getting individual speeds
double forward = chassisSpeeds.vxMetersPerSecond;
double sideways = chassisSpeeds.vyMetersPerSecond;
double angular = chassisSpeeds.omegaRadiansPerSecond;
   
    // Now use this in our kinematics
 

	boolean isSlowMode = false;

 
	public double getRightSlaveEncoderDistance() {
        return rightSlave.getSelectedSensorPosition()
                * (1.0 / TrajectoryConstants.kEncoderCPR)
                * (Math.PI * TrajectoryConstants.kWheelDiameterMeters);
    }

    public double getLeftSlaveEncoderDistance() {
        return leftSlave.getSelectedSensorPosition()
                * (1.0 / TrajectoryConstants.kEncoderCPR)
                * (-Math.PI * TrajectoryConstants.kWheelDiameterMeters);
    }
    public double getRightMasterEncoderDistance() {
      return rightMaster.getSelectedSensorPosition()
              * (1.0 / TrajectoryConstants.kEncoderCPR)
              * (Math.PI * TrajectoryConstants.kWheelDiameterMeters);
  }
  public double getLeftMasterEncoderDistance() {
    return leftMaster.getSelectedSensorPosition()
            * (1.0 / TrajectoryConstants.kEncoderCPR)
            * (Math.PI * TrajectoryConstants.kWheelDiameterMeters);
}

	double Deadband(double i){
		return ((i >= +0.09) ? i : ((i <= -0.09) ? i : 0));
	}
	public void changeSlowMode(){
		if (isSlowMode){
			isSlowMode = false;
		}else{
			isSlowMode = true;
		}
	}
}