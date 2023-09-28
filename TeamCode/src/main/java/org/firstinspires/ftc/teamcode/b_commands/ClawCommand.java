package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

public class ClawCommand extends CommandBase {

	private final ClawSubsystem           claw;
	private final ClawSubsystem.ClawState state;

	public ClawCommand(ClawSubsystem claw, ClawSubsystem.ClawState state) {
		this.claw  = claw;
		this.state = state;

		addRequirements(claw);
	}

	@Override
	public void execute() {
		if (state == ClawSubsystem.ClawState.CLOSE) {
			claw.close();
		} else {
			claw.open();
		}
	}

	@Override
	public boolean isFinished() {
		return claw.atAngle();
	}

}
