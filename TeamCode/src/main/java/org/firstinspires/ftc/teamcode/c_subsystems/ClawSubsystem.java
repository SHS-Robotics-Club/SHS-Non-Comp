package org.firstinspires.ftc.teamcode.c_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class ClawSubsystem extends SubsystemBase {
	private final ServoEx clawLeft;
	private final ServoEx clawRight;
	public static ClawState clawState    = ClawState.INIT;

	/**
	 * @param clawLeft  The Left claw servo object.
	 * @param clawRight The Right claw servo object.
	 */
	public ClawSubsystem(ServoEx clawLeft, ServoEx clawRight) {
		this.clawLeft  = clawLeft;
		this.clawRight = clawRight;

		init();
	}

	@Override
	public void periodic() {
		setStateAngle();
	}

	public void toggle(){
		if (clawState == ClawState.OPEN) {
			clawState = ClawState.CLOSE;
		} else if (clawState == ClawState.CLOSE){
			clawState = ClawState.OPEN;
		}
	}

	public void setStateAngle (){
		setAngle(clawState.getStateAngle());
	}

	public void setAngle(double angle) {
		clawLeft.turnToAngle(angle);
		clawRight.turnToAngle(angle);
	}

	public double getAngle() {
		return clawLeft.getAngle();
	}

	/**
	 * Checks if the lift is where it should be.
	 */
	public boolean atAngle() {
		return getAngle() == clawState.getStateAngle();
	}

	public void open() {
		clawState = ClawState.OPEN;
	}

	public void close() {
		clawState = ClawState.CLOSE;
	}

	public void init() {
		clawState = ClawState.INIT;
	}

	public boolean isOpen() {
		return clawState == ClawState.OPEN;
	}

	public enum ClawState {
		OPEN(23), CLOSE(-15), INIT(30);

		private final int clawAng;

		ClawState(int clawAng) {
			this.clawAng = clawAng;
		}

		public int getStateAngle() {
			return clawAng;
		}
	}
}