package org.firstinspires.ftc.teamcode.b_commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

public class ParkCommand extends CommandBase {
	private static final int startX = 36, startY = 65, startH = 90;
	private final MecanumSubsystem  drive;
	private final AprilTagSubsystem tagSubsystem;
	StartingZone startingZone;

	public ParkCommand(MecanumSubsystem drive, AprilTagSubsystem tagSubsystem,
	                   StartingZone startingZone) {
		this.drive        = drive;
		this.tagSubsystem = tagSubsystem;
		this.startingZone = startingZone;

		//addRequirements(drive);
	}

	@Override
	public void initialize() {
		TrajectorySequence park =
				getParkTrajectory(startingZone.getStartPose(), tagSubsystem.getParkingZone());
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

	public TrajectorySequence getParkTrajectory(Pose2d startPose,
	                                            AprilTagSubsystem.ParkingZone zone) {

		//startPose = new Pose2d(0, 0, Math.toRadians(0));

		drive.setPoseEstimate(startPose);

		switch (zone) {
			case LEFT:
				return drive.trajectorySequenceBuilder(startPose)
				            .forward(24)
				            .strafeLeft(24)
				            .build();
			case RIGHT:
				return drive.trajectorySequenceBuilder(startPose)
				            .forward(24)
				            .strafeRight(24)
				            .build();
			default:
				return drive.trajectorySequenceBuilder(startPose)
				            .forward(24)
				            .build();
		}
	}

	public enum StartingZone {
		RED_LEFT(-startX, -startY, startH),
		RED_RIGHT(startX, -startY, startH),
		BLUE_LEFT(startX, startY, -startH),
		BLUE_RIGHT(-startX, startY, -startH);

		private final int X, Y, H;

		StartingZone(int X, int Y, int H) {
			this.X = X;
			this.Y = Y;
			this.H = H;
		}

		public Pose2d getStartPose() {
			return new Pose2d(X, Y, Math.toRadians(H));
		}
	}
}
