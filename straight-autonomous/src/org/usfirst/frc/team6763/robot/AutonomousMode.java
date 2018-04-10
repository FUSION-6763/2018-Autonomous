/**
 * Interface to hold autonomous mode instructions.
 */

package org.usfirst.frc.team6763.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.usfirst.frc.team6763.robot.Instruction;
import org.usfirst.frc.team6763.robot.Instruction.State;

public interface AutonomousMode {
	// state, distance/time(limit), angle
	List<Instruction> switchLeftPositionLeft = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 75, 0),
			new Instruction(State.SPIN_RIGHT, 0, 45),
			new Instruction(State.RAISE_LIFT, 1, 0),
			new Instruction(State.DRIVE_FORWARD, 8, 45),
			new Instruction(State.WAIT, 1, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> switchLeftPositionCenter = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 94, -50),
			new Instruction(State.SPIN_RIGHT, 0, 0),
			new Instruction(State.RAISE_LIFT, 1.2, 0),
			new Instruction(State.DRIVE_FORWARD, 8, 0),
			new Instruction(State.WAIT, 1, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> switchLeftPositionRight = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 60, 0),
			new Instruction(State.SPIN_LEFT, 0, -90),
			new Instruction(State.DRIVE_FORWARD, 148, 90),
			new Instruction(State.SPIN_RIGHT, 0, 0),
			new Instruction(State.RAISE_LIFT, 1, 0),
			new Instruction(State.DRIVE_FORWARD, 29, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> switchRightPositionLeft = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 60, 0),
			new Instruction(State.TURN_RIGHT, 0, 90),
			new Instruction(State.DRIVE_FORWARD, 148, 90),
			new Instruction(State.TURN_LEFT, 0, 0),
			new Instruction(State.RAISE_LIFT, 1, 0),
			new Instruction(State.DRIVE_FORWARD, 29, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> switchRightPositionCenter = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 52, 35),
			new Instruction(State.SPIN_LEFT, 0, 0),
			new Instruction(State.RAISE_LIFT, 1, 0),
			new Instruction(State.DRIVE_FORWARD, 20, 0),
			new Instruction(State.WAIT, 1, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> switchRightPositionRight = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 75, 0),
			new Instruction(State.TURN_LEFT, 0, -45),
			new Instruction(State.RAISE_LIFT, 1, 0),
			new Instruction(State.DRIVE_FORWARD, 8, -45),
			new Instruction(State.WAIT, 1, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> scaleLeftPositionLeft = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 230, 0),
			new Instruction(State.SPIN_RIGHT, 0, 30),
			new Instruction(State.RAISE_LIFT, 2.5, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> scaleLeftPositionCenter = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 36, 0)));
	
	List<Instruction> scaleLeftPositionRight = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 200, 0),
			new Instruction(State.TURN_LEFT, 0, -90),
			new Instruction(State.DRIVE_FORWARD, 188, -90),
			new Instruction(State.TURN_RIGHT, 0, 13),
			new Instruction(State.DRIVE_FORWARD, 5, 13),
			new Instruction(State.RAISE_LIFT, 2.5, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> scaleRightPositionLeft = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 200, 0),
			new Instruction(State.SPIN_RIGHT, 0, 90),
			new Instruction(State.DRIVE_FORWARD, 188, 90),
			new Instruction(State.SPIN_LEFT, 0, -13),
			new Instruction(State.DRIVE_FORWARD, 5, 13),
			new Instruction(State.RAISE_LIFT, 2.5, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> scaleRightPositionCenter = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 36, 0)));
	
	List<Instruction> scaleRightPositionRight = new ArrayList<Instruction>(Arrays.asList(
			new Instruction(State.DRIVE_FORWARD, 230, 0),
			new Instruction(State.TURN_LEFT, 0, -30),
			new Instruction(State.RAISE_LIFT, 2.5, 0),
			new Instruction(State.EJECT_CUBE, 1, 0)));
	
	List<Instruction> stop = new ArrayList<Instruction>(Arrays.asList(new Instruction(State.STOP, 0, 0)));
}
