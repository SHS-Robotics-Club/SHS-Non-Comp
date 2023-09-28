package org.firstinspires.ftc.teamcode.b_commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

public class TrajectoryFollowerCommand extends CommandBase {

	private final MecanumSubsystem   drive;
	private final TrajectorySequence trajectory;

	public TrajectoryFollowerCommand(MecanumSubsystem drive, TrajectorySequence trajectory) {
		this.drive      = drive;
		this.trajectory = trajectory;

		addRequirements(drive);
	}

	@Override
	public void initialize() {
		drive.followTrajectorySequenceAsync(trajectory);
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

}
