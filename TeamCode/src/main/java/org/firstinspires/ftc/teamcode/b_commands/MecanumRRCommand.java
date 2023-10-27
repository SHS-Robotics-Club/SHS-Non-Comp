package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumRRSubsystem;

import java.util.function.DoubleSupplier;

public class MecanumRRCommand extends CommandBase {
    private final MecanumRRSubsystem drive;
    private final DoubleSupplier     forwardInput, strafeInput, turnInput;
    private double multiplier = 1.0;

    /**
     * Constructor for MecanumCommand with a default multiplier of 1.0.
     *
     * @param drive   The drive subsystem this command will run on.
     * @param forward The control input for driving forwards/backwards.
     * @param strafe  The control input for driving left/right.
     * @param turn    The control input for turning.
     */
    public MecanumRRCommand(MecanumRRSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        this.drive        = drive;
        this.forwardInput = forward;
        this.strafeInput  = strafe;
        this.turnInput    = turn;

        addRequirements(drive);
    }

    /**
     * Constructor for MecanumCommand with a custom multiplier.
     *
     * @param drive      The drive subsystem this command will run on.
     * @param forward    The control input for driving forwards/backwards.
     * @param strafe     The control input for driving left/right.
     * @param turn       The control input for turning.
     * @param multiplier A multiplier for robot speed.
     */
    public MecanumRRCommand(MecanumRRSubsystem drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn, double multiplier) {
        this(drive, forward, strafe, turn);
        this.multiplier = multiplier;
    }

    @Override
    public void execute() {
        double forwardValue = applyDeadzone(-forwardInput.getAsDouble() * 0.9 * multiplier, 0.1);
        double turnValue    = applyDeadzone(turnInput.getAsDouble() * 0.8 * multiplier, 0.1);
        double strafeValue  = applyDeadzone(strafeInput.getAsDouble() * 0.9 * multiplier, 0.15);

        drive.drive(forwardValue, strafeValue, turnValue);
    }

    private double applyDeadzone(double input, double deadzone) {
        return Math.abs(input) < deadzone ? 0 : clipRange(squareInput(input));
    }

    private double clipRange(double value) {
        return value <= -1.0 ? -1.0 : Math.min(value, 1.0);
    }

    private double squareInput(double input) {
        return input * Math.abs(input);
    }
}