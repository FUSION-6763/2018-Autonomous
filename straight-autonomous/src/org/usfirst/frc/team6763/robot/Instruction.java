package org.usfirst.frc.team6763.robot;

public class Instruction {
	
	private State _state;
	private double _limit;
	private float _targetAngle;
	
	/**
	 * Creates a new Instruction instance.
	 */
	public Instruction(final State state, final double limit, final float targetAngle) {
		_state = state;
		_limit = limit;
		_targetAngle = targetAngle;
	}
	
	public State getState() {
		return _state;
	}
	
	public double getLimit() {
		return _limit;
	}
	
	public float getTargetAngle() {
		return _targetAngle;
	}
	
	/**
	 * Robot abilities
	 */
	public enum State {
		STOP,
		DRIVE_FORWARD,
		DRIVE_BACKWARD,
		TURN_RIGHT,
		TURN_LEFT,
		RAISE_LIFT,
		LOWER_LIFT,
		EJECT_CUBE,
		PULL_CUBE,
		WAIT;
	}
}