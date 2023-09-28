package org.firstinspires.ftc.teamcode.b_commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

public class ParkCommandWithout extends CommandBase {
	private static final int startX = 36, startY = 65, startH = 90;
	private final MecanumSubsystem  drive;
	private final AprilTagSubsystem tagSubsystem;
	private final Pose2d            startPose;

	public ParkCommandWithout(MecanumSubsystem drive, AprilTagSubsystem tagSubsystem,
	                          Pose2d startPose) {
		this.drive        = drive;
		this.tagSubsystem = tagSubsystem;
		this.startPose    = startPose;

		//addRequirements(drive);
	}

	@Override
	public void initialize() {
		TrajectorySequence park =
				getParkTrajectory(startPose, tagSubsystem.getParkingZone());
		drive.followTrajectorySequence(park);
	}

	@Override
	public void execute() {
		drive.update();
	}

	@Override
	public void end(boolean interrupted) {
		if (interrupted) {
			drive.stop();
		}
	}

	@Override
	public boolean isFinished() {
		return Thread.currentThread().isInterrupted() || !drive.isBusy();
	}

	public TrajectorySequence getParkTrajectory(Pose2d startPose, AprilTagSubsystem.ParkingZone zone) {

		drive.setPoseEstimate(startPose);

		switch (zone) {
			case LEFT:
				return drive.trajectorySequenceBuilder(startPose)
				            .strafeLeft(24)
				            .build();
			case RIGHT:
				return drive.trajectorySequenceBuilder(startPose)
				            .strafeRight(24)
				            .build();
			default:
				return drive.trajectorySequenceBuilder(startPose)
				            .back(0.01)
				            .build();
		}
	}
}
