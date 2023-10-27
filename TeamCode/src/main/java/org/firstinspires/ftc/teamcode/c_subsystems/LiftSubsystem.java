package org.firstinspires.ftc.teamcode.c_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;

/**
 * The LiftSubsystem class represents the robot's lifting mechanism.
 * It controls the movement of the lift using PIDF control.
 */
@Config
public class LiftSubsystem extends SubsystemBase {
    // Constants for lift math
    private static final double SPOOL_CIRCUMFERENCE    = Math.PI / 30; //mm
    private static final double LIFT_CONVERSION_FACTOR = SPOOL_CIRCUMFERENCE / 537.7; //Ticks

    // Ticks per Millimeter = (Encoder Resolution × Gear Ratio) / Distance per Revolution in Millimeters

    // PIDF coefficients
    private static final double KP = 0.01;
    private static final double KI = 0.0;
    private static final double KD = 0.0001;
    private static final double KF = 0.0;

    // Tolerance values
    private static final double POSITION_TOLERANCE = 5.0;
    private static final double VELOCITY_TOLERANCE = 1.0;

    // Other parameters
    private static LiftLevels LIFT_LEVEL = LiftLevels.FLOOR;
    private static int        MODIFIER   = 0;
    private static boolean    LOWERED    = false, MODIFIER_HALTED = false;

    // Motors and controllers
    private final MotorGroup     lift;
    private final CRServo        spool;
    private final PIDFController pidfController = new PIDFController(KP, KI, KD, KF);

    /**
     * Constructs a LiftSubsystem object.
     *
     * @param lift  The motor group controlling the lift.
     * @param spool The servo controlling the lift spool.
     */
    public LiftSubsystem(MotorGroup lift, CRServo spool) {
        this.lift  = lift;
        this.spool = spool;
        moveToFloor();
        setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
    }
    @Override
    public void periodic() {
        updateSpool();
        calculate();
        updateLift();
    }

    /**
     * Checks if the lift is in a lowered state.
     *
     * @return True if the lift is lowered, false otherwise.
     */
    public static boolean isLowered() {
        return LOWERED;
    }

    /**
     * Lowers or raises the lift based on the given boolean value.
     *
     * @param lower True to lower the lift, false to raise the lift.
     */
    public void lower(boolean lower) {
        LOWERED = lower;
    }

    /**
     * Updates the spool servo position based on the lift's setpoint.
     */
    public void updateSpool() {
        if (getSetPoint() >= 800) {
            setSpool(0);
        } else {
            setSpool(1);
        }
    }

    /**
     * Sets the position of the spool servo.
     *
     * @param output The desired position of the spool servo.
     */
    public void setSpool(double output) {
        spool.set(output);
    }

    /**
     * Sets the power to the lift motors.
     *
     * @param output The desired power for the lift motors.
     */
    public void setLiftPower(double output) {
        lift.set(output);
    }

    // Encoder and PID control methods

    /**
     * Stops the lift motors.
     */
    public void stopLift() {
        setLiftPower(0);
        lift.stopMotor();
    }

    /**
     * @return the velocity of the lift motors in ticks per second
     */
    public double getVelocity() {
        return lift.getVelocities().get(0);
    }

    /**
     * @return the current position of the lift motors in ticks
     */
    public Double getPosition() {
        return lift.getPositions().get(0);
    }

    /**
     * @return the positional error e(t)
     */
    public double getPositionError() {
        return pidfController.getPositionError();
    }

    /**
     * Resets the lift motor encoders without stopping the motors.
     */
    public void resetEncoder() {
        lift.resetEncoder();
    }

    /**
     * Resets the PIDF controller.
     */
    public void resetPIDF() {
        pidfController.reset();
    }

    /**
     * Sets the error tolerances for the PIDF controller.
     *
     * @param positionTolerance The position error tolerance.
     * @param velocityTolerance The velocity error tolerance.
     */
    public void setTolerance(double positionTolerance, double velocityTolerance) {
        pidfController.setTolerance(positionTolerance, velocityTolerance);
    }

    /**
     * Sets the error tolerances for the PIDF controller.
     *
     * @param positionTolerance The position error tolerance.
     */
    public void setTolerance(double positionTolerance) {
        pidfController.setTolerance(positionTolerance);
    }

    /**
     * Gets the current setpoint of the PIDF controller.
     *
     * @return The current setpoint.
     */
    public double getSetPoint() {
        return pidfController.getSetPoint();
    }

    /**
     * Sets the setpoint for the PIDF controller.
     *
     * @param sp The desired setpoint.
     */
    public void setSetPoint(double sp) {
        pidfController.setSetPoint(sp);
    }

    /**
     * Sets the lift height in millimeters.
     *
     * @param heightInMM The desired height in millimeters.
     */
    public void setHeightInMM(double heightInMM) {
        // Calculate the target position in ticks based on millimeters and lift conversion factor
        double targetPosition = heightInMM / LIFT_CONVERSION_FACTOR;

        // Set the calculated position as the setpoint for the PIDF controller
        setSetPoint(targetPosition);
    }

    // Lift control logic methods

    /**
     * Checks if the lift is at the setpoint within the tolerance range.
     *
     * @return Whether the lift is at the setpoint.
     */
    public boolean atSetPoint() {
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

    // Lift modifier methods

    /**
     * Updates the lift motor position and power based on the current setpoint and error.
     */
    public void updateLift() {
        setSetPoint(LIFT_LEVEL.calculatePosition());

        if (atSetPoint()) {
            stopLift();
        } else {
            setLiftPower(calculate());
        }
    }

    /**
     * Resets the lift modifier.
     */
    public void resetModifier() {
        MODIFIER = 0;
    }

    /**
     * Gets the current lift modifier value.
     *
     * @return The lift modifier.
     */
    public int getModifier() {
        return MODIFIER;
    }

    /**
     * Increases the lift modifier by the given amount if not halted.
     *
     * @param amount The amount to increase the modifier by.
     */
    public void increaseModifier(int amount) {
        if (!MODIFIER_HALTED) {
            MODIFIER += amount;
        }
    }

    /**
     * Decreases the lift modifier by the given amount if not halted.
     *
     * @param amount The amount to decrease the modifier by.
     */
    public void decreaseModifier(int amount) {
        if (!MODIFIER_HALTED) {
            MODIFIER -= amount;
        }
    }

    // Lift level control methods

    /**
     * Sets the lift level to the floor.
     */
    public void moveToFloor() {
        LIFT_LEVEL = LiftLevels.FLOOR;
        resetModifier();
    }

    /**
     * Sets the lift level to low.
     */
    public void moveToLow() {
        LIFT_LEVEL = LiftLevels.LOW;
        resetModifier();
    }

    /**
     * Sets the lift level to medium.
     */
    public void moveToMedium() {
        LIFT_LEVEL = LiftLevels.MED;
        resetModifier();
    }

    /**
     * Sets the lift level to high.
     */
    public void moveToHigh() {
        LIFT_LEVEL = LiftLevels.HIGH;
        resetModifier();
    }

    /**
     * Gets the drive multiplier associated with the current lift level.
     *
     * @return The drive multiplier.
     */
    public double getDriveMultiplier() {
        return LIFT_LEVEL.getDriveMultiplier();
    }

    // Coefficient methods

    /**
     * Gets the PIDF coefficients.
     *
     * @return The PIDF coefficients in an array [kP, kI, kD, kF].
     */
    public double[] getCoefficients() {
        return pidfController.getCoefficients();
    }

    /**
     * Sets the PIDF coefficients.
     *
     * @param kP The proportional coefficient.
     * @param kI The integral coefficient.
     * @param kD The derivative coefficient.
     * @param kF The feedforward coefficient.
     */
    public void setCoefficients(double kP, double kI, double kD, double kF) {
        pidfController.setPIDF(kP, kI, kD, kF);
    }

    /**
     * Enumeration representing different lift levels.
     */
    public enum LiftLevels {
        FLOOR(0, 1.0), LOW(920, 1.0), MED(1256, 0.8), HIGH(1590, 0.7);

        private final int    LEVEL_POSITION;
        private final double DRIVE_MULTIPLIER;

        LiftLevels(int levelPosition, double driveMultiplier) {
            this.LEVEL_POSITION   = levelPosition;
            this.DRIVE_MULTIPLIER = driveMultiplier;
        }

        /**
         * Calculates the target position of the lift based on the current level.
         *
         * @return The target position of the lift motor.
         */
        public int calculatePosition() {
            int finalPosition;
            if (LiftSubsystem.isLowered()) {
                finalPosition = LEVEL_POSITION + LiftSubsystem.MODIFIER - 200;
            } else {
                finalPosition = LEVEL_POSITION + LiftSubsystem.MODIFIER;
            }

            if (finalPosition > HIGH.LEVEL_POSITION) {
                LiftSubsystem.MODIFIER_HALTED = true;
                LiftSubsystem.MODIFIER        = HIGH.LEVEL_POSITION - LEVEL_POSITION;
                return HIGH.LEVEL_POSITION;
            } else if (finalPosition < FLOOR.LEVEL_POSITION) {
                LiftSubsystem.MODIFIER_HALTED = true;
                LiftSubsystem.MODIFIER        = LEVEL_POSITION - FLOOR.LEVEL_POSITION;
                return FLOOR.LEVEL_POSITION;
            } else {
                MODIFIER_HALTED = false;
                return finalPosition;
            }
        }

        /**
         * Gets the drive multiplier associated with the current level.
         *
         * @return The drive multiplier.
         */
        public double getDriveMultiplier() {
            return DRIVE_MULTIPLIER;
        }
    }
}
