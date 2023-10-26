package org.firstinspires.ftc.teamcode.c_subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

/**
 * Represents a subsystem controlling the robot's claw mechanism.
 * The ClawSubsystem manages the open and close states of the claw using two servos.
 */
public class ClawSubsystem extends SubsystemBase {
    public static ClawState clawState;
    private final ServoEx   leftClawServo;
    private final ServoEx   rightClawServo;

    /**
     * Constructor for ClawSubsystem.
     *
     * @param leftClawServo  The left claw servo object.
     * @param rightClawServo The right claw servo object.
     */
    public ClawSubsystem(ServoEx leftClawServo, ServoEx rightClawServo) {
        this.leftClawServo  = leftClawServo;
        this.rightClawServo = rightClawServo;
        clawState           = ClawState.INIT;
    }

    @Override
    public void periodic() {
        updateStateAngle();
    }

    /**
     * Toggles the claw state between OPEN and CLOSE.
     */
    public void toggleClaw() {
        clawState = (clawState == ClawState.OPEN) ? ClawState.CLOSE : ClawState.OPEN;
        setClawAngle(clawState.getStateAngle());
    }

    /**
     * Sets the claw servo angles based on the current claw state.
     */
    public void updateStateAngle() {
        setClawAngle(clawState.getStateAngle());
    }

    /**
     * Gets the current angle of the claw servos.
     *
     * @return The current angle of the claw servos.
     */
    public double getClawAngle() {
        return leftClawServo.getAngle();
    }

    /**
     * Sets the claw servo angles to a specific angle.
     *
     * @param angle The desired angle for the claw servos.
     */
    public void setClawAngle(double angle) {
        leftClawServo.turnToAngle(angle);
        rightClawServo.turnToAngle(angle);
    }

    /**
     * Checks if the claw is at the desired angle.
     *
     * @return True if the claw is at the desired angle, false otherwise.
     */
    public boolean isAtAngle() {
        return getClawAngle() == clawState.getStateAngle();
    }

    /**
     * Opens the claw.
     */
    public void openClaw() {
        clawState = ClawState.OPEN;
    }

    /**
     * Closes the claw.
     */
    public void closeClaw() {
        clawState = ClawState.CLOSE;
    }

    /**
     * Checks if the claw is in the open state.
     *
     * @return True if the claw is open, false otherwise.
     */
    public boolean isOpen() {
        return clawState == ClawState.OPEN;
    }

    /**
     * Enum representing the different states of the claw.
     */
    public enum ClawState {
        OPEN(23), CLOSE(-15), INIT(30);

        private final int clawAngle;

        ClawState(int clawAngle) {
            this.clawAngle = clawAngle;
        }

        /**
         * Gets the angle associated with the claw state.
         *
         * @return The angle of the claw for the specific state.
         */
        public int getStateAngle() {
            return clawAngle;
        }
    }
}
