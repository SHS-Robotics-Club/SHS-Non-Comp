package org.firstinspires.ftc.teamcode.c_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class LiftSubsystem extends SubsystemBase {
	public static final double liftMath = 29.8 * Math.PI / 537.7; //0.174110809001
	public static       double kP       = 0.01, kI = 0.0, kD = 0.0001, kF = 0.0;
	public static double positionTolerance = 5.0, velocityTolerance = 1.0;
	public static double     ffVelocity = 50;
	public static LiftLevels liftLevels = LiftLevels.FLOOR;
	public static int        modifier   = 0;
	static        boolean    lower      = false, modHalt = false;
	MotorGroup     lift;
	CRServo        spool;
	PIDFController pidf = new PIDFController(kP, kI, kD, kF);

	public LiftSubsystem(MotorGroup lift, CRServo spool) {
		this.lift  = lift;
		this.spool = spool;

		floor();
		setTolerance(positionTolerance, velocityTolerance);
	}

	@Override
	public void periodic() {
		updateSpool();
		calculate();
		updateLift();
	}

	public void lower(boolean lower) {
		this.lower = lower;
	}

	public void updateSpool() {
		if (getSetPoint() >= 800) {
			setSpool(0);
		} else {
			setSpool(1);
		}
	}

	public void setSpool(double output) {
		spool.set(output);
	}

	public void set(double output) {
		lift.set(output);
	}

	public void stop() {
		set(0);
		lift.stopMotor();
	}

	/**
	 * @return the velocity of the motor in ticks per second
	 */
	public double getVelocity() {
		return lift.getVelocities().get(0);
	}

	/**
	 * @return the current position of the encoder
	 */
	public Double getPosition() {
		return lift.getPositions().get(0);
	}

	/**
	 * Resets the encoder without having to stop the motor.
	 */
	public void resetEncoder() {
		lift.resetEncoder();
	}

	/**
	 * Resets pid loop.
	 */
	public void resetPIF() {
		pidf.reset();
	}

	/**
	 * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
	 *
	 * @param positionTolerance Position error which is tolerable.
	 * @param velocityTolerance Velocity error which is tolerable.
	 */
	public void setTolerance(double positionTolerance, double velocityTolerance) {
		pidf.setTolerance(positionTolerance, velocityTolerance);
	}

	/**
	 * Sets the error which is considered tolerable for use with {@link #atSetPoint()}.
	 *
	 * @param positionTolerance Position error which is tolerable.
	 */
	public void setTolerance(double positionTolerance) {
		pidf.setTolerance(positionTolerance);
	}

	/**
	 * Returns the current setpoint of the PIDFController.
	 *
	 * @return The current setpoint.
	 */
	public double getSetPoint() {
		return pidf.getSetPoint();
	}

	/**
	 * Sets the setpoint for the PIDFController
	 *
	 * @param sp The desired setpoint.
	 */
	public void setSetPoint(double sp) {
		pidf.setSetPoint(sp);
	}

	/**
	 * Returns true if the error is within the percentage of the total input range, determined by
	 * {@link #setTolerance}.
	 *
	 * @return Whether the error is within the acceptable bounds.
	 */
	public boolean atSetPoint() {
		return pidf.atSetPoint();
	}

/*	public double tickToMil(double ticks) {
		return ticks * liftMath;
	}

	public double milToTicks(double mil) {
		return mil / liftMath;
	}*/

	/**
	 * @return the positional error e(t)
	 */
	public double getPositionError() {
		return pidf.getPositionError();
	}

	/**
	 * @return the velocity error e'(t)
	 */
	public double getVelocityError() {
		return pidf.getVelocityError();
	}

	/**
	 * Calculates the control value, u(t).
	 *
	 * @return the value produced by u(t).
	 */
	public double calculate() {
		//pidf.setF(eff.calculate(ffVelocity));
		return pidf.calculate(getPosition());
	}

	/**
	 * Checks if the lift is where it should be.
	 */
	public void updateLift() {
		setSetPoint(liftLevels.calculatePosition());

		getPosition();
		if (atSetPoint()) {
			stop();
		} else if (!atSetPoint()) {
			set(calculate());
		}
	}

	public static boolean checkLowered() {
		return lower;
	}

	public void resetMod() {
		modifier = 0;
	}

	public int getMod() {
		return modifier;
	}

	public void up(int ammount) {
		if (!modHalt) {
			modifier += ammount;
		}

	}

	public void down(int ammount) {
		if (!modHalt) {
			modifier -= ammount;
		}
	}

	public void floor() {
		liftLevels = LiftLevels.FLOOR;
		resetMod();
	}

	public void low() {
		liftLevels = LiftLevels.LOW;
		resetMod();
	}

	public void med() {
		liftLevels = LiftLevels.MED;
		resetMod();
	}

	public void high() {
		liftLevels = LiftLevels.HIGH;
		resetMod();
	}

	/**
	 * @return the PIDF coefficients
	 */
	public double[] getCoefficients() {
		return pidf.getCoefficients();
	}

	public void setCoefficients(double kP, double kI, double kD, double kF){
		pidf.setPIDF(kP, kI, kD, kF);
	}

	public enum LiftLevels {
		FLOOR(0, 1.0), LOW(920, 1.0), MED(1256, 0.8), HIGH(1590, 0.7);

		private final int    levelPos;
		private final double driveMult;

		LiftLevels(int levelPos, double driveMult) {
			this.levelPos  = levelPos;
			this.driveMult = driveMult;
		}

		public int calculatePosition() {
			int finalPos;
			if (LiftSubsystem.checkLowered()) {
				finalPos = levelPos + LiftSubsystem.modifier - 200;
			} else {
				finalPos = levelPos + LiftSubsystem.modifier;
			}

			if (finalPos > HIGH.levelPos) {
				LiftSubsystem.modHalt  = true;
				LiftSubsystem.modifier = HIGH.levelPos - levelPos;
				return HIGH.levelPos;
			} else if (finalPos < FLOOR.levelPos) {
				LiftSubsystem.modHalt  = true;
				LiftSubsystem.modifier = levelPos - FLOOR.levelPos;
				return FLOOR.levelPos;
			} else {
				LiftSubsystem.modHalt = false;
				return finalPos;
			}
		}

		public double getDriveMult() {
			return driveMult;
		}
	}
}