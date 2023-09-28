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
 * A subsystem that uses the {@link MecanumDrive} class.
 * This periodically calls {@link MecanumDrive#update()} which runs the internal
 * state machine for the mecanum drive. All movement/following is async to fit the paradigm.
 */
public class MecanumSubsystem extends SubsystemBase {

	private final MecanumDrive drive;
	private final boolean          fieldCentric;

	public MecanumSubsystem(MecanumDrive drive, boolean isFieldCentric) {
		this.drive   = drive;
		fieldCentric = isFieldCentric;
	}

	public void updatePoseEstimate() {
		drive.updatePoseEstimate();
	}

	public void drive(double leftY, double leftX, double rightX) {
		Pose2d poseEstimate = getLocalizer().getPoseEstimate();

		Vector2d input =
				new Vector2d(-leftY, -leftX).rotated(fieldCentric ? -poseEstimate.getHeading() : 0);

		setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -rightX));
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

	public void setMotorPowers(double v, double v1, double v2, double v3) {
		drive.setMotorPowers(v, v1, v2, v3);
	}

	public double getRawExternalHeading() {
		return drive.getRawExternalHeading();
	}

}