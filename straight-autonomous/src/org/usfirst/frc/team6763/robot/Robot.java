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
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.hal.HAL;
import edu.wpi.first.wpilibj.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

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
	
	Spark elevator1 = new Spark(3);
	Spark elevator2 = new Spark(4);
	
	Spark intakeL = new Spark(7);
	Spark intakeR = new Spark(8);
	
	JoystickButton X = new JoystickButton(stick, 3);
	JoystickButton Y = new JoystickButton(stick, 4);
	
	JoystickButton bumperR = new JoystickButton(stick, 6);
	
	Encoder leftEncoder = new Encoder(0, 1);
	Encoder rightEncoder = new Encoder(2, 3);
	
	Timer timer = new Timer();
	
	//Spark climber = new Spark(7);
	
	AHRS navx = new AHRS(SerialPort.Port.kUSB);
	
	String data;
	
	final float ticksPerInch = 53;
	double defaultSpeed;
	
	boolean firstRun[];
	int mode = 1;
	
	float lastEncoderValue;
	double lastTimerValue;
	
	
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
		leftEncoder.setReverseDirection(true);
		
		SmartDashboard.putNumber("Default Speed", 0.75);
		
		elevator1.setInverted(true);
		intakeL.setInverted(true);
		
		leftEncoder.reset();
		rightEncoder.reset();
	}
	
	public void robotPeriodic() {
		SmartDashboard.putNumber("Gyro", navx.getYaw());
		
		//System.out.println(leftEncoder.get());
		System.out.println("Left Encoder: "+leftEncoder.get());
		System.out.println("Right Encoder: "+rightEncoder.get());
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
		
		timer.reset();
		timer.stop();
		
		mode = 1;
		
		firstRun = new boolean[] {true, true, true, true, true, true, true, true, true, true, true};
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
						if(leftEncoder.get() < ticksPerInch * 244) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
							intakeL.set(0.1);
							intakeR.set(0.1);
						}
						else if(navx.getYaw() < 17) {
							myRobot.tankDrive(defaultSpeed, -defaultSpeed);
						}
						else if(firstRun[0]) {
							lastEncoderValue = leftEncoder.get();
							firstRun[0] = false;
						}
						else if(leftEncoder.get() < lastEncoderValue + (ticksPerInch * 5)) {
							accurateDrive(navx.getYaw(), defaultSpeed, 17, 2);
						}
						else if(firstRun[1]) {
							timer.start();
							firstRun[1] = false;
						}
						else if(timer.get() < 3.25) {
							elevator1.set(1.0);
							elevator2.set(1.0);
							myRobot.tankDrive(0.0, 0.0);
						}
						else if(timer.get() < 3.75) {
							elevator1.set(0.0);
							elevator2.set(0.0);
						}
						else if(timer.get() < 4.75) {
							elevator1.set(0.0);
							elevator2.set(0.0);
							intakeL.set(0.7);
							intakeR.set(0.7);
						}
						else {
							intakeL.set(0.0);
							intakeR.set(0.0);
							elevator1.set(0.0);
							elevator2.set(0.0);
							myRobot.tankDrive(0.0, 0.0);
						}
						//elevator1.set(0.3);
						//elevator2.set(0.3);
					}
					else {
						// Right side of scale
						if(mode == 1 && leftEncoder.get() < ticksPerInch * 215){
							//Drive straight 224"
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
							intakeL.set(0.1);
							intakeR.set(0.1);
						}
						else if(firstRun[0]){
							//Next mode
							mode++;
							firstRun[0] = false;
						}
						else if(mode == 2 && navx.getYaw() < 87){
							//Turn right 90 degrees
							myRobot.tankDrive(defaultSpeed * 0.95, -(defaultSpeed * 0.95));
						}
						else if(firstRun[1]){
							//Next mode, reset encoders
							mode++;
							leftEncoder.reset();
							firstRun[1] = false;
						}
						else if(mode == 3 && leftEncoder.get() < ticksPerInch * 197){
							//Drive straight 224"
							accurateDrive(navx.getYaw(), defaultSpeed, 93, 2);
						}
						else if(firstRun[2]){
							//Next mode
							mode++;
							firstRun[2] = false;
						}
						else if(mode == 4 && navx.getYaw() > -17){
							myRobot.tankDrive(-(defaultSpeed * 0.95), defaultSpeed * 0.95);
						}
						else if(firstRun[3]){
							//Next mode, reset encoders
							mode++;
							leftEncoder.reset();
							firstRun[3] = false;
						}
						else if(mode == 5 && leftEncoder.get() < ticksPerInch * 14) {
							accurateDrive(navx.getYaw(), defaultSpeed, -25, 2);
						}
						else if(firstRun[4]) {
							timer.start();
							firstRun[4] = false;
						}
						else if(timer.get() < 3.25) {
							elevator1.set(1.0);
							elevator2.set(1.0);
							myRobot.tankDrive(0.0, 0.0);
						}
						else if(timer.get() < 3.75) {
							elevator1.set(0.0);
							elevator2.set(0.0);
						}
						else if(timer.get() < 4.75) {
							elevator1.set(0.0);
							elevator2.set(0.0);
							intakeL.set(0.7);
							intakeR.set(0.7);
						}
						else {
							intakeL.set(0.0);
							intakeR.set(0.0);
							elevator1.set(0.0);
							elevator2.set(0.0);
							myRobot.tankDrive(0.0, 0.0);
						}
					}
				}
				else if(HAL.getAllianceStation() == AllianceStationID.Blue2 || HAL.getAllianceStation() == AllianceStationID.Red2) {
					// Station 2 code
					if(data.charAt(1) == 'L') {
						// Left side of scale
						if(leftEncoder.get() < ticksPerInch * 76) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
							intakeL.set(0.1);
							intakeR.set(0.1);
						}
						else if(navx.getYaw() > -87) {
							myRobot.tankDrive(-(defaultSpeed / 2), defaultSpeed / 2);
						}
						else if(firstRun[0]) {
							lastEncoderValue = leftEncoder.get();
							firstRun[0] = false;
						}
						else if(leftEncoder.get() < lastEncoderValue + (ticksPerInch * 91)) {
							accurateDrive(navx.getYaw(), defaultSpeed, -90, 2);
						}
						else if(navx.getYaw() < 0) {
							myRobot.tankDrive(defaultSpeed / 2, -(defaultSpeed / 2));
						}
						else if(firstRun[1]) {
							lastEncoderValue = leftEncoder.get();
							firstRun[1] = false;
						}
						else if(leftEncoder.get() < lastEncoderValue + (ticksPerInch * 252)) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
						}
						else if(navx.getYaw() < 87) {
							myRobot.tankDrive(defaultSpeed / 2, -(defaultSpeed / 2));
						}
						else {
							myRobot.tankDrive(0.0, 0.0);
						}
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
							intakeL.set(0.1);
							intakeR.set(0.1);
						}
						else if(navx.getYaw() > -87) {
							myRobot.tankDrive(-(defaultSpeed / 2), defaultSpeed / 2);
						}
						else {
							intakeL.set(0.0);
							intakeR.set(0.0);
							myRobot.tankDrive(0.0, 0.0);
						}
					}
				}
				break;
			case "switch":
				// Put switch auto code here
				if(HAL.getAllianceStation() == AllianceStationID.Blue1 || HAL.getAllianceStation() == AllianceStationID.Red1) {
					//Station 1 code
					if(data.charAt(0) == 'L') {
						//Left side of switch
						if(leftEncoder.get() < ticksPerInch * 100) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
							intakeL.set(0.1);
							intakeR.set(0.1);
						}
						else if(navx.getYaw() < 17) {
							myRobot.tankDrive(defaultSpeed, -defaultSpeed);
						}
						else if(firstRun[1]) {
							timer.start();
							firstRun[1] = false;
						}
						else if(timer.get() < 1.25) {
							elevator1.set(1.0);
							elevator2.set(1.0);
							myRobot.tankDrive(0.0, 0.0);
						}
						else if(timer.get() < 1.75) {
							elevator1.set(0.0);
							elevator2.set(0.0);
						}
						else if(firstRun[2]) {
							lastEncoderValue = leftEncoder.get();
							firstRun[2] = false;
						}
						else if(leftEncoder.get() < lastEncoderValue + 800) {
							myRobot.tankDrive(defaultSpeed * 0.8, defaultSpeed * 0.8);
						}
						else if(firstRun[3]) {
							lastTimerValue = timer.get();
							firstRun[3] = false;
						}
						else if(timer.get() < lastTimerValue + 1.0) {
							elevator1.set(0.0);
							elevator2.set(0.0);
							intakeL.set(0.5);
							intakeR.set(0.5);
						}
						else {
							intakeL.set(0.0);
							intakeR.set(0.0);
							elevator1.set(0.0);
							elevator2.set(0.0);
							myRobot.tankDrive(0.0, 0.0);
						}
					}
					else {
						//Right side of switch
						
					}
				}
				else if(HAL.getAllianceStation() == AllianceStationID.Blue2 || HAL.getAllianceStation() == AllianceStationID.Red2) {
					//Station 2 code
					if(data.charAt(0) == 'L') {
						//Left side of switch
						if(leftEncoder.get() < ticksPerInch * 62.72) {
							//Drive straight 62.72"
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
							intakeL.set(0.1);
							intakeR.set(0.1);
						}
						else if(navx.getYaw() > -87) {
							//Turn left 90 degrees
							myRobot.tankDrive(-(defaultSpeed / 2), defaultSpeed / 2);
						}
						else if(firstRun[0]) {
							//Reset encoders
							lastEncoderValue = leftEncoder.get();
							firstRun[0] = false;
						}
						else if(leftEncoder.get() < lastEncoderValue + (ticksPerInch * 34.72)) {
							//Drive straight 34.72"
							accurateDrive(navx.getYaw(), defaultSpeed, -90, 2);
						}
						else if(navx.getYaw() < 0) {
							//Turn right 90 degrees
							myRobot.tankDrive(defaultSpeed / 2, -(defaultSpeed / 2));
						}
						else if(firstRun[1]) {
							lastEncoderValue = leftEncoder.get();
							firstRun[1] = false;
						}
						else if(leftEncoder.get() < lastEncoderValue + (ticksPerInch * 69.44)) {
							//Drive straight 69.44"
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
						}
						else {
							//Stop
							myRobot.tankDrive(0.0, 0.0);
						}
					}
					else {
						//Right side of switch
						
					}
				}
				else if(HAL.getAllianceStation() == AllianceStationID.Blue3 || HAL.getAllianceStation() == AllianceStationID.Red3) {
					//Station 3 code
					if(data.charAt(0) == 'L') {
						//Left side of switch
						
					}
					else {
						//Right side of switch
						if(leftEncoder.get() < ticksPerInch * 168) {
							accurateDrive(navx.getYaw(), defaultSpeed, 0, 2);
							intakeL.set(0.1);
							intakeR.set(0.1);
						}
						else if(navx.getYaw() > -87) {
							myRobot.tankDrive(-(defaultSpeed / 2), defaultSpeed / 2);
						}
						else {
							myRobot.tankDrive(0.0, 0.0);
							intakeL.set(0.0);
							intakeR.set(0.0);
						}
					}
				}
				break;
		}
		System.out.println(mode);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		if(bumperR.get()) {
			myRobot.arcadeDrive(-(stick.getY() / 2), stick.getX() / 2);
		}
		else {
			myRobot.arcadeDrive(-stick.getY(), stick.getX());
		}
		
		//myRobot.tankDrive(-stick.getY(), -stick.getRawAxis(5));
		
		//climber.set(-stick.getRawAxis(3));
		
		elevator1.set(-stick.getRawAxis(5));
		elevator2.set(-stick.getRawAxis(5));
		
		if(X.get()) {
			intakeL.set(-1.0);
			intakeR.set(-1.0);
		}
		else if(Y.get()) {
			intakeL.set(1.0);
			intakeR.set(1.0);
		}
		else {
			intakeL.set(0.0);
			intakeR.set(0.0);
		}
	}
	
	@Override
	public void testInit() {
		leftEncoder.reset();
		rightEncoder.reset();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		System.out.println("Left Encoder: "+leftEncoder.get());
		System.out.println("Right Encoder: "+rightEncoder.get());
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
	public void disabledPeriodic() {
		//System.out.println("Left Encoder: "+leftEncoder.get());
		//System.out.println("Right Encoder: "+rightEncoder.get());
	}
}
