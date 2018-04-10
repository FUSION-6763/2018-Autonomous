/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6763.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SerialPort;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team6763.robot.Instruction;
import org.usfirst.frc.team6763.robot.Instruction.State;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends IterativeRobot {
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	SendableChooser<String> stationChooser = new SendableChooser<>();
	
	final DifferentialDrive myRobot = new DifferentialDrive(new Spark(0), new Spark(2));
	final Joystick stick = new Joystick(0);
	final Joystick driveStick = new Joystick(1);
			
	Spark elevator1 = new Spark(3);
	Spark elevator2 = new Spark(4);
	
	Spark intakeL = new Spark(7);
	Spark intakeR = new Spark(8);
	
	JoystickButton A = new JoystickButton(stick, 1);
	JoystickButton B = new JoystickButton(stick, 2);
	JoystickButton X = new JoystickButton(stick, 3);
	JoystickButton Y = new JoystickButton(stick, 4);
	JoystickButton RB = new JoystickButton(stick, 6);
	
	JoystickButton bumperR = new JoystickButton(driveStick, 6);
	
	Encoder leftEncoder = new Encoder(0, 1);
	Encoder rightEncoder = new Encoder(2, 3);
	
	Timer timer = new Timer();
	
	PowerDistributionPanel PDP = new PowerDistributionPanel();
	
	double currentSpeed;
	double previousSpeed;
	final double accelerationRate = .006;
	final double decelerationRate = .005;
	final double scaleOutput = 0.7;
	final double switchOutput = 0.5; 
	final double distanceScale = 0.8;
	final double minimumDriveSpeed = 0.6;
	final double startSpeed = 0.3;
	final double robotWidth = 27;
	
	final static int tolerance = 2;
	final AHRS navx = new AHRS(SerialPort.Port.kUSB);
	
	String data; 
	
	final float ticksPerInch = 53;
	double defaultSpeed = 1.0;
	
	State state = State.STOP;
	List<Instruction> autoMode = new ArrayList<Instruction>();
	int instructionIndex = 0;
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Scale", "scale");
		m_chooser.addObject("Switch", "switch");
		SmartDashboard.putData("Auto choices", m_chooser);
		
		stationChooser.addDefault("Station 1", "station1");
		stationChooser.addObject("Station 2", "station2");
		stationChooser.addObject("Station 3", "station3");
		SmartDashboard.putData("Station Choices", stationChooser);
		
		rightEncoder.setReverseDirection(true);
		leftEncoder.setReverseDirection(true);
		
		SmartDashboard.putNumber("Default Speed", defaultSpeed);
		
		CameraServer.getInstance().startAutomaticCapture();
		
		elevator1.setInverted(true);
		intakeL.setInverted(true);
		
		leftEncoder.reset();
		rightEncoder.reset();
	}
		
	
	public void robotPeriodic() {
		SmartDashboard.putNumber("Gyro", navx.getYaw());
	}

	@Override
	public void autonomousInit() {
		System.out.println("Auto selected: " + m_chooser.getSelected());
		
		currentSpeed = startSpeed;
		instructionIndex = 0;
		
		navx.reset();
		leftEncoder.reset();
		rightEncoder.reset();
		
		data = DriverStation.getInstance().getGameSpecificMessage();
		
		defaultSpeed = SmartDashboard.getNumber("Default Speed", defaultSpeed);
		
		timer.stop();
		timer.reset();
		
		// Determine if the scale is on the right or left.
		if (m_chooser.getSelected().equals("scale")) {
			
			// Determine which alliance station the robot is in.
			if (data.charAt(1) == 'R') {
				
				// Determine which alliance station the robot is in.
				switch (stationChooser.getSelected()) {
				case "station1":
					autoMode = AutonomousMode.scaleRightPositionLeft;
					break;
				case "station2":
					autoMode = AutonomousMode.scaleRightPositionCenter;
					break;
				default:
					autoMode = AutonomousMode.scaleRightPositionRight;
					break;
				}
			} else {
		
				//	Determine which alliance station the robot is in.
				switch (stationChooser.getSelected()) {
					case "station1":
						autoMode = AutonomousMode.scaleLeftPositionLeft;
						System.out.println("Picked Auto Mode");
						break;
					case "station2":
						autoMode = AutonomousMode.scaleLeftPositionCenter;
						break;
					default:
						autoMode = AutonomousMode.scaleLeftPositionRight;
						break;
				}
			}
		} else if (m_chooser.getSelected().equals("switch")) {
		
			// Determine if the switch is on the left or right.
			if (data.charAt(0) == 'R') {
			
				// Determine which alliance station the robot is in.
				switch (stationChooser.getSelected()) {
				case "station1":
					autoMode = AutonomousMode.switchRightPositionLeft;
					break;
				case "staiton2":
					autoMode = AutonomousMode.switchRightPositionCenter;
					break;
				default:
					autoMode = AutonomousMode.switchRightPositionRight;
					break;
				}
			}
			else {
				switch (stationChooser.getSelected()) {
				case "station1":
					autoMode = AutonomousMode.switchLeftPositionLeft;
					break;
				case "station2":
					autoMode = AutonomousMode.switchLeftPositionCenter;
					break;
				default:
					autoMode = AutonomousMode.switchLeftPositionRight;
					break;
				}
			}
		} else {
			//Set to default mode (STOP)
			autoMode = AutonomousMode.stop;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Instruction instruction = new Instruction(State.STOP, 0, 0);
		if(instructionIndex < autoMode.size()) {
			instruction = autoMode.get(instructionIndex);
		}
		
		final float currentAngle = navx.getYaw();
		
		switch (instruction.getState()) {
		
			case DRIVE_FORWARD:
				if (getDistanceTraveled() < instruction.getLimit()) {
					if(currentSpeed < defaultSpeed && getDistanceTraveled() / ticksPerInch < distanceScale * instruction.getLimit() && currentSpeed > minimumDriveSpeed) {
						currentSpeed += accelerationRate;
					} else if(getDistanceTraveled() / ticksPerInch > distanceScale * instruction.getLimit() && currentSpeed > minimumDriveSpeed) {
						currentSpeed -= accelerationRate;
					}
					accurateDrive(navx.getYaw(), currentSpeed, instruction.getTargetAngle(), tolerance);
				}
				else {
					currentSpeed = startSpeed;
					leftEncoder.reset();
					rightEncoder.reset();
					instructionIndex++;
				}
				break;
				
			case DRIVE_BACKWARD:
				if (getDistanceTraveled() < instruction.getLimit()) {
					if(currentSpeed < defaultSpeed && getDistanceTraveled() / ticksPerInch < distanceScale * instruction.getLimit()) {
						currentSpeed+=accelerationRate;
					}
					else if(getDistanceTraveled() / ticksPerInch > distanceScale * instruction.getLimit() && currentSpeed > minimumDriveSpeed) {
						currentSpeed-=accelerationRate;
					}
					accurateDrive(navx.getYaw(), -currentSpeed, instruction.getTargetAngle(), tolerance);
				}
				else {
					currentSpeed = startSpeed;
					leftEncoder.reset();
					rightEncoder.reset();
					instructionIndex++;
				}
				break;
				
			case SPIN_RIGHT:
				System.out.println("Robot Turning Right");
				if (currentSpeed <= startSpeed) {
					currentSpeed = defaultSpeed;
				}
				if (currentAngle < instruction.getTargetAngle() - tolerance ||
					currentAngle > instruction.getTargetAngle() + tolerance) {
					if(currentSpeed > 0.55 && currentAngle > instruction.getTargetAngle() * 0.5) {
						currentSpeed -= decelerationRate;
					}
					myRobot.tankDrive(currentSpeed, -currentSpeed);
				} else {
					currentSpeed = startSpeed;
					leftEncoder.reset();
					rightEncoder.reset();
					instructionIndex++;
				}
				break;
				
			case SPIN_LEFT:
				if(currentSpeed <= startSpeed) {
					currentSpeed = defaultSpeed;
				}
				if (currentAngle < instruction.getTargetAngle() - tolerance ||
					currentAngle > instruction.getTargetAngle() + tolerance) {
					if(currentSpeed > 0.55 && currentAngle < instruction.getTargetAngle() * 0.5) {
						currentSpeed -= decelerationRate;
					}
					myRobot.tankDrive(-currentSpeed, currentSpeed);
				} else {
					currentSpeed = startSpeed;
					leftEncoder.reset();
					rightEncoder.reset();
					instructionIndex++;
				}
				break;
				
			case TURN_RIGHT:
				if (currentSpeed <= startSpeed) {
					currentSpeed = defaultSpeed;
				}
				
				if (currentAngle < instruction.getTargetAngle() - tolerance ||
					currentAngle > instruction.getTargetAngle() + tolerance) {
					if (currentSpeed > 0.55 && currentAngle > instruction.getTargetAngle() * 0.8) {
						currentSpeed -= decelerationRate;
					}
					myRobot.tankDrive(currentSpeed, -getInnerCurveSpeed(currentSpeed, instruction.getLimit()));
				} else {
					currentSpeed = startSpeed;
					leftEncoder.reset();
					rightEncoder.reset();
					instructionIndex++;
				}
				break;
				
			case TURN_LEFT:
				if (currentSpeed <= startSpeed) {
					currentSpeed = defaultSpeed;
				}
				
				if (currentAngle < instruction.getTargetAngle() - tolerance ||
					currentAngle > instruction.getTargetAngle() + tolerance) {
					if (currentSpeed > 0.55 && currentAngle > instruction.getTargetAngle() * 0.8) {
						currentSpeed -= decelerationRate;
					}
					myRobot.tankDrive(currentSpeed, -getInnerCurveSpeed(currentSpeed, instruction.getLimit()));
				} else {
					currentSpeed = startSpeed;
					leftEncoder.reset();
					rightEncoder.reset();
					instructionIndex++;
				}
				break;
				
			case RAISE_LIFT:
				if (Double.compare(timer.get(), 0) == 0) {
					timer.start();
				}
				System.out.println("Time: "+timer.get());
				if(timer.get() < instruction.getLimit()) {
					if(currentSpeed < 1) {
						currentSpeed+=accelerationRate;
					}
					elevator1.set(currentSpeed);
					elevator2.set(currentSpeed);
					myRobot.tankDrive(0.0, 0.0);
				}
				else {
					leftEncoder.reset();
					rightEncoder.reset();
					timer.stop();
					timer.reset();
					elevator1.set(0.0);
					elevator2.set(0.0);
					instructionIndex++;
					currentSpeed = startSpeed;
				}
				break;
				
			case LOWER_LIFT:
				if (Double.compare(timer.get(), 0) == 0) {
					timer.start();
				}
				System.out.println("Time: "+timer.get());
				if(timer.get() < instruction.getLimit()) {
					elevator1.set(1.0);
					elevator2.set(1.0);
					myRobot.tankDrive(0.0,0.0);
				}
				else {
					leftEncoder.reset();
					rightEncoder.reset();
					timer.stop();
					timer.reset();
					elevator1.set(0.0);
					elevator2.set(0.0);
					instructionIndex++;
					currentSpeed = startSpeed;
				}
				
			case EJECT_CUBE:
				if (Double.compare(timer.get(), 0) == 0) {
					timer.start();
				}
				if(timer.get() < instruction.getLimit()) {
					intakeL.set((m_chooser.getSelected().equals("switch")) ? switchOutput : scaleOutput);
					intakeR.set((m_chooser.getSelected().equals("switch")) ? switchOutput : scaleOutput);
				}
				else {
					timer.stop();
					timer.reset();
					intakeL.set(-0.2);
					intakeR.set(-0.2);
					instructionIndex++;
				}
				break;
				
			case WAIT:
				if (Double.compare(timer.get(), 0) == 0) {
					timer.start();
				}
				if (timer.get() < instruction.getLimit()) {
					Timer.delay(1);
				} else {
					timer.stop();
					timer.reset();
					instructionIndex++;
				}
				break;
			case STOP: // intentional fall-through
			default:
				myRobot.tankDrive(0, 0);
				elevator1.set(0.0);
				elevator2.set(0.0);
				break;
		}
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	
	public void teleopInit() {
		
	}
	
	public void teleopPeriodic() {
		if(bumperR.get()) {
			myRobot.arcadeDrive(-(driveStick.getY()*0.6), driveStick.getX()*0.6, false);
		}
		else {
			myRobot.arcadeDrive(-driveStick.getY(), driveStick.getX(), false);
		}
		
		elevator1.set(-stick.getRawAxis(1) / 1.5);
		elevator2.set(-stick.getRawAxis(1) / 1.5);
				
		if(X.get()) {
			intakeL.set(-0.7);
			intakeR.set(-0.7);
		}
		else if(Y.get()) {
			intakeL.set(0.7);
			intakeR.set(0.7);
		}
		else if(A.get()) {
			intakeL.set(-0.7);
			intakeR.set(0.7);
		}
		else if(B.get()) {
			intakeL.set(0.7);
			intakeR.set(-0.7);
		}
		else if(RB.get()) {
			intakeL.set(0.35);
			intakeR.set(0.35);
		}
		else {
			intakeL.set(-0.2);
			intakeR.set(-0.2);
		}
	}
	
	@Override
	public void testInit() {

	}
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
	}
	
	public void accurateDrive(final float gyroValue, final double speed, final double targetAngle, final int tolerance) {
		System.out.println("speed: "+speed);
		if(gyroValue < targetAngle - tolerance) {
			System.out.println("Too far left");
			myRobot.tankDrive(speed, speed / 4);
		}
		else if(gyroValue > targetAngle + tolerance) {
			System.out.println("Too far right");
			myRobot.tankDrive(speed / 4, speed);
		}
		else {
			System.out.println("Good");
			myRobot.tankDrive(speed, speed);
		}
	}
	
	private float getDistanceTraveled() {
		final float leftDistance = leftEncoder.get() / ticksPerInch;
		final float rightDistance = rightEncoder.get() / ticksPerInch;
		return (leftDistance + rightDistance) / 2f;
	}
	
	private double getInnerCurveSpeed(final double outerSpeed, final double radius) {
		return outerSpeed * ((radius - (robotWidth / 2)) / (radius + robotWidth / 2));
	}
}
