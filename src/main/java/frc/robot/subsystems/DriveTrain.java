// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.SPI;


import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.TrajectoryConstants;

public class DriveTrain extends SubsystemBase {
	enum SpeedMode{
		Low,
		Default,
		High
	}
	//private SpeedMode speedMode = SpeedMode.Default;

	private WPI_VictorSPX motor_solOn;
	private WPI_VictorSPX motor_solArka;
	private WPI_VictorSPX motor_sagOn;
	private WPI_VictorSPX motor_sagArka;

	private MecanumDrive m_drive;
  private PS4Controller m_stick= new PS4Controller(JoystickConstants.driverPort);
	public final MecanumDriveOdometry m_odometry;
    public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	private final Field2d m_field = new Field2d();
	public DriveTrain() {

        WPI_VictorSPX motor_solOn = new WPI_VictorSPX(01);
        WPI_VictorSPX motor_solArka = new WPI_VictorSPX(02);
        WPI_VictorSPX motor_sagOn = new WPI_VictorSPX(03);
        WPI_VictorSPX motor_sagArka = new WPI_VictorSPX(04);

		motor_sagOn.setInverted(true);
        motor_sagArka.setInverted(true);



		m_drive = new MecanumDrive(motor_solOn, motor_solArka,motor_sagOn,motor_sagArka);
		
		m_odometry = new MecanumDriveOdometry(TrajectoryConstants.kDriveKinematics, m_gyro.getRotation2d(), new MecanumDriveWheelPositions());
        SmartDashboard.putData("Field", m_field);
		zeroHeading();
        resetEncoders();
	}

public void Drive(){	
	m_drive.driveCartesian(-m_stick.getLeftY(), m_stick.getLeftX(), -m_stick.getRawAxis(2));

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
        motor_solOn.setSelectedSensorPosition(0);
        motor_sagOn.setSelectedSensorPosition(0);
		motor_sagArka.setSelectedSensorPosition(0);
        motor_solArka.setSelectedSensorPosition(0);
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
        return motor_sagArka.getSelectedSensorPosition()
                * (1.0 / TrajectoryConstants.kEncoderCPR)
                * (Math.PI * TrajectoryConstants.kWheelDiameterMeters);
    }

    public double getLeftSlaveEncoderDistance() {
        return motor_solArka.getSelectedSensorPosition()
                * (1.0 / TrajectoryConstants.kEncoderCPR)
                * (-Math.PI * TrajectoryConstants.kWheelDiameterMeters);
    }
    public double getRightMasterEncoderDistance() {
      return motor_sagOn.getSelectedSensorPosition()
              * (1.0 / TrajectoryConstants.kEncoderCPR)
              * (Math.PI * TrajectoryConstants.kWheelDiameterMeters);
  }
  public double getLeftMasterEncoderDistance() {
    return motor_solOn.getSelectedSensorPosition()
            * (1.0 / TrajectoryConstants.kEncoderCPR)
            * (Math.PI * TrajectoryConstants.kWheelDiameterMeters);
}
	public void changeSlowMode(){
		if (isSlowMode){
			isSlowMode = false;
		}else{
			isSlowMode = true;
		}
	}
}