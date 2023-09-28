package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumCommand extends CommandBase {

	private static double forwardOut;
	private final MecanumSubsystem drive;
	private final DoubleSupplier   forward, strafe, turn;
	private        double multiplier;

	/**
	 * @param drive   The drive subsystem this command wil run on.
	 * @param strafe  The control input for driving left/right.
	 * @param forward The control input for driving forwards/backwards.
	 * @param turn    The control input for turning.
	 */
	public MecanumCommand(MecanumSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
		this.drive   = drive;
		this.strafe  = strafe;
		this.forward = forward;
		this.turn    = turn;
		multiplier   = 1.0;

		addRequirements(drive);
	}

	/**
	 * @param drive      The drive subsystem this command wil run on.
	 * @param strafe     The control input for driving left/right.
	 * @param forward    The control input for driving forwards/backwards.
	 * @param turn       The control input for turning.
	 * @param multiplier A multiplier for bot speed.
	 */
	public MecanumCommand(MecanumSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, double multiplier) {
		this.drive      = drive;
		this.strafe     = strafe;
		this.forward    = forward;
		this.turn       = turn;
		this.multiplier = multiplier;

		addRequirements(drive);
	}

	public static double returnForward() {
		return forwardOut;
	}

	public static double applyDeadzoneExponentiation(double input, double deadzone, double exponent) {
		double magnitude = Math.abs(input);
		if (magnitude < deadzone) {
			return 0;
		}
		return Math.signum(input) * Math.pow(magnitude - deadzone, exponent) /
		       (1 - Math.pow(deadzone, exponent));
	}

	@Override
	public void execute() {
		double forwardMult = (-forward.getAsDouble() * 0.9) * multiplier;
		double turnMult    = (turn.getAsDouble() * 0.8) * multiplier;
		double strafeMult  = (strafe.getAsDouble() * 0.9) * multiplier;

		double forwardValue = applyDeadzone(forwardMult, 0.1);
		double turnValue    = applyDeadzone(turnMult, 0.1);
		double strafeValue  = applyDeadzone(strafeMult, 0.15);

		forwardOut = forwardValue;

		drive.drive(forwardValue, strafeValue, turnValue);
	}

	public double applyDeadzone(double input, double deadzone) {
		if (Math.abs(input) < deadzone) {
			return 0;
		}

		return clipRange(squareInput(input));
	}

	/**
	 * Returns minimum range value if the given value is less than
	 * the set minimum. If the value is greater than the set maximum,
	 * then the method returns the maximum value.
	 *
	 * @param value The value to clip.
	 */
	public double clipRange(double value) {
		return value <= -1.0 ? -1.0 : value >= 1.0 ? 1.0 : value;
	}

	/**
	 * Square magnitude of number while keeping the sign.
	 */
	protected double squareInput(double input) {
		return input * Math.abs(input);
	}

}