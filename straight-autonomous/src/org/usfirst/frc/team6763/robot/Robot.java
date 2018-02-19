/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6763.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;

import com.kauailabs.navx.frc.AHRS;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	DifferentialDrive myRobot = new DifferentialDrive(new Spark(0), new Spark(2));
	Joystick stick = new Joystick(0);
	Encoder leftEncoder = new Encoder(0, 1);
	Encoder rightEncoder = new Encoder(2, 3);
	
	Spark climber = new Spark(7);
	
	AHRS navx = new AHRS(SerialPort.Port.kUSB);
	
	String data;
	
	final int ticksPerInch = 106;
	double defaultSpeed;
	
	boolean firstRun = true;
	float lastEncoderValue;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Scale", "scale");
		m_chooser.addObject("Switch", "switch");
		SmartDashboard.putData("Auto choices", m_chooser);
		
		rightEncoder.setReverseDirection(true);
		
		SmartDashboard.putNumber("Default Speed", 0.5);
	}
	
	public void robotPeriodic() {
		SmartDashboard.putNumber("Gyro", navx.getYaw());
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + m_chooser.getSelected());
		
		navx.reset();
		leftEncoder.reset();
		rightEncoder.reset();
		
		data = DriverStation.getInstance().getGameSpecificMessage();
		
		defaultSpeed = SmartDashboard.getNumber("Default Speed", 0.5);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_chooser.getSelected()) {
			case "scale":
				// Put scale auto code here
				if(HAL.getAllianceStation() == AllianceStationID.Blue1 || HAL.getAllianceStation() == AllianceStationID.Red1) {
					// Station 1 code
					if(data.charAt(1) == 'L') {
						// Left side of scale
						if(leftEncoder.get() < ticksPerInch * 321) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
						}
						else if(navx.getYaw() < 87) {
							myRobot.tankDrive(defaultSpeed, 0.0);
						}
						else {
							myRobot.tankDrive(0.0, 0.0);
						}
					}
					else {
						// Right side of scale
						if(leftEncoder.get() < ticksPerInch * 224) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
						}
						else if(navx.getYaw() < 87) {
							myRobot.tankDrive(defaultSpeed, 0.0);
						}
						else if(firstRun) {
							lastEncoderValue = leftEncoder.get();
							firstRun = false;
						}
						else if(leftEncoder.get() < lastEncoderValue + (ticksPerInch * 100)) {
							accurateDrive(navx.getYaw(), defaultSpeed, 90, 2);
							System.out.println((leftEncoder.get() - lastEncoderValue) / ticksPerInch);
						}
						else {
							myRobot.tankDrive(0.0, 0.0);
						}
					}
				}
				else if(HAL.getAllianceStation() == AllianceStationID.Blue2 || HAL.getAllianceStation() == AllianceStationID.Red2) {
					// Station 2 code
					if(data.charAt(1) == 'L') {
						// Left side of scale
						
					}
					else {
						// Right side of scale
						
					}
				}
				else if(HAL.getAllianceStation() == AllianceStationID.Blue3 || HAL.getAllianceStation() == AllianceStationID.Red3) {
					// Station 3 code
					if(data.charAt(1) == 'L') {
						// Left side of scale
						
					}
					else {
						// Right side of scale
						if(leftEncoder.get() < ticksPerInch * 336) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
						}
						else if(navx.getYaw() > -87) {
							myRobot.tankDrive(0.0, defaultSpeed);
						}
						else {
							myRobot.tankDrive(0.0, 0.0);
						}
					}
				}
				break;
			case "switch":
				// Put switch auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		myRobot.arcadeDrive(-stick.getY(), stick.getX());
		
		climber.set(-stick.getRawAxis(3));
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	public void accurateDrive(float gyroValue, double speed, double targetAngle, int tolerence) {
		if(gyroValue < targetAngle - tolerence) {
			System.out.println("Too far left");
			myRobot.tankDrive(speed, speed / 4);
		}
		else if(gyroValue > targetAngle + tolerence) {
			System.out.println("Too far right");
			myRobot.tankDrive(speed / 4, speed);
		}
		else {
			System.out.println("Good");
			myRobot.tankDrive(speed, speed);
		}
	}
}
