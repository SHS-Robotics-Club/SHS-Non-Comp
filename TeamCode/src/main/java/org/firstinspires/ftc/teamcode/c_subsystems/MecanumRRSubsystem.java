package org.firstinspires.ftc.teamcode.c_subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.d_roadrunner.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.List;

/**
 * A subsystem utilizing the {@link MecanumDrive} class. Handles robot movement and localization.
 */
public class MecanumRRSubsystem extends SubsystemBase {

    private final MecanumDrive drive;
    private final boolean      isFieldCentric;

    /**
     * Constructor for MecanumSubsystem.
     *
     * @param drive          The MecanumDrive object controlling robot movement.
     * @param isFieldCentric A boolean indicating if the robot operates in field-centric mode.
     */
    public MecanumRRSubsystem(MecanumDrive drive, boolean isFieldCentric) {
        this.drive          = drive;
        this.isFieldCentric = isFieldCentric;
    }

    /**
     * Updates the robot's pose estimate based on its odometry readings.
     */
    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    /**
     * Drives the robot using the given joystick inputs.
     *
     * @param leftY  The left joystick Y input.
     * @param leftX  The left joystick X input.
     * @param rightX The right joystick X input.
     */
    public void drive(double leftY, double leftX, double rightX) {
        Pose2d   poseEstimate = this.getLocalizer().getPoseEstimate();
        Vector2d input        = new Vector2d(-leftY, -leftX).rotated(isFieldCentric ? -poseEstimate.getHeading() : 0);
        setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -rightX));
    }

    public void turn180(){
        drive.turn(Math.toRadians(180) - 1e-6);
    }

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return drive.trajectoryBuilder(startPose, startHeading);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose);
    }

    public void turnAsync(double angle) {
        drive.turnAsync(angle);
    }

    public void turn(double angle) {
        drive.turn(angle);
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectory(trajectory);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequence(trajectorySequence);
    }

    public Pose2d getLastError() {
        return drive.getLastError();
    }

    public void update() {
        drive.update();
    }

    public void waitForIdle() {
        drive.waitForIdle();
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        drive.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        drive.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(runMode, coefficients);
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        drive.setWeightedDrivePower(drivePower);
    }

    public List<Double> getWheelPositions() {
        return drive.getWheelPositions();
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    /**
     * Sets the motor powers for the robot's drive motors.
     *
     * @param frontLeft  Front left motor power.
     * @param rearLeft   Rear left motor power.
     * @param frontRight Front right motor power.
     * @param rearRight  Rear right motor power.
     */
    public void setMotorPowers(double frontLeft, double rearLeft, double frontRight, double rearRight) {
        drive.setMotorPowers(frontLeft, rearLeft, frontRight, rearRight);
    }

    /**
     * Gets the raw external heading of the robot.
     *
     * @return The raw external heading.
     */
    public double getRawExternalHeading() {
        return drive.getRawExternalHeading();
    }
}