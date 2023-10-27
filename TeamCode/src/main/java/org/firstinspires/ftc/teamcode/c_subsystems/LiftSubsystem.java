package org.firstinspires.ftc.teamcode.c_subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

/**
 * The LiftSubsystem class represents the robot's lifting mechanism.
 * It controls the movement of the lift using PIDF control.
 */
@Config
public class LiftSubsystem extends SubsystemBase {
    // Constants for lift math
    private static final double SPOOL_CIRCUMFERENCE    = Math.PI / 30; // in mm
    private static final double LIFT_CONVERSION_FACTOR = SPOOL_CIRCUMFERENCE / 537.7; // in Ticks per Millimeter

    // Ticks per Millimeter = (Encoder Resolution × Gear Ratio) / Distance per Revolution in Millimeters

    // PIDF coefficients
    private static final double KP = 0.01;
    private static final double KI = 0.0;
    private static final double KD = 0.0001;
    private static final double KF = 0.0;

    // Tolerance values
    private static final double POSITION_TOLERANCE = 5.0;
    private static final double VELOCITY_TOLERANCE = 1.0;
    private static final double LIFT_MAX_HEIGHT    = 300; // in mm (not rn)

    // Motors and controllers
    private final MotorGroup   lift;
    private final PIDFController pidfController  = new PIDFController(KP, KI, KD, KF);

    /**
     * Constructs a LiftSubsystem object.
     *
     * @param lift The motor group controlling the lift.
     */
    public LiftSubsystem(MotorGroup lift) {
        this.lift = lift;
        moveToPreset(Presets.GROUND);
        setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    }

    @Override
    public void periodic() {
        correctPosition();
    }

    /**
     * Sets the power to the lift motors.
     *
     * @param power The desired power for the lift motors (-1 to 1).
     */
    public void setPower(double power) {
        lift.set(power);
    }

    /**
     * Stops the lift motors.
     */
    public void stop() {
        setPower(0);
        lift.stopMotor();
    }

    /**
     * Resets the lift motor encoders without stopping the motors.
     */
    public void resetEncoder() {
        lift.resetEncoder();
    }

    /**
     * Sets the error tolerances for the PIDF controller.
     *
     * @param positionTolerance The position error tolerance in ticks.
     * @param velocityTolerance The velocity error tolerance in ticks per second.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        pidfController.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Gets the current setpoint of the PIDF controller.
     *
     * @return The current setpoint in ticks.
     */
    public double getSetpoint() {
        return pidfController.getSetPoint();
    }

    /**
     * Gets the current position of the lift motors in ticks.
     *
     * @return The current position of the lift motors in ticks.
     */
    public double getPosition() {
        return lift.getPositions().get(0);
    }

    /**
     * Gets the velocity of the lift motors in ticks per second.
     *
     * @return The velocity of the lift motors in ticks per second.
     */
    public double getVelocity() {
        return lift.getVelocities().get(0);
    }

    /**
     * Gets the positional error e(t) of the PIDF controller.
     *
     * @return The positional error e(t) of the PIDF controller.
     */
    public double getPositionError() {
        return pidfController.getPositionError();
    }

    /**
     * Sets the setpoint for the PIDF controller.
     *
     * @param setpoint The desired setpoint in ticks.
     */
    public void setSetpoint(double setpoint) {
        double clampedSetpoint = clamp(setpoint, 0, LIFT_MAX_HEIGHT / LIFT_CONVERSION_FACTOR);
        pidfController.setSetPoint(clampedSetpoint);
    }

    /**
     * Sets the lift height in millimeters.
     *
     * @param heightInMM The desired height in millimeters.
     */
    public void setHeightInMM(double heightInMM) {
        double targetPosition = heightInMM / LIFT_CONVERSION_FACTOR;
        setSetpoint(targetPosition);
    }

    /**
     * Checks if the lift is at the setpoint within the tolerance range.
     *
     * @return Whether the lift is at the setpoint.
     */
    public boolean atSetpoint() {
        return pidfController.atSetPoint();
    }

    /**
     * Calculates the control value for the lift motors using PIDF control.
     *
     * @return The control value.
     */
    public double calculate() {
        return pidfController.calculate(getPosition());
    }

    /**
     * Updates the lift motor position and power based on the current setpoint and error.
     */
    public void correctPosition() {
        if (atSetpoint()) {
            stop();
        } else {
            setPower(calculate());
        }
    }

    /**
     * Moves the lift to the specified preset height.
     *
     * @param preset The target preset height (e.g., Presets.GROUND, Presets.LOW, etc.).
     */
    public void moveToPreset(Presets preset) {
        setSetpoint(preset.getPresetHeight());
    }

    /**
     * Adjusts the lift position based on the modifier.
     *
     * @param modifier The target change of height.
     */
    public void modifyPosition(int modifier) {
        setSetpoint(getSetpoint() + modifier);
    }


    public void setCoefficients(double kP, double kI, double kD, double kF){
        pidfController.setPIDF(kP, kI, kD, kF);
    }

    /**
     * Calculates the percentage of how much the lift is extended.
     *
     * @return The percentage the lift is at (from 0.0 to 1.0).
     */
    public double getPercentage() {
        return getPosition() / LIFT_MAX_HEIGHT;
    }

    /**
     * Enumeration representing different lift presets.
     */
    public enum Presets {
        GROUND(0), LOW(920), MEDIUM(1256), HIGH(1590);

        private final int PRESET_HEIGHT;

        Presets(int height) {
            this.PRESET_HEIGHT = height;
        }

        /**
         * Gets the preset height in ticks.
         *
         * @return The preset height in ticks.
         */
        public int getPresetHeight() {
            return PRESET_HEIGHT;
        }
    }
}
