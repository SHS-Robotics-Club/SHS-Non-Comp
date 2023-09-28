package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;

public class LiftCommand extends CommandBase {

	private final LiftSubsystem            lift;
	private final LiftSubsystem.LiftLevels state;

	public LiftCommand(LiftSubsystem lift, LiftSubsystem.LiftLevels state) {
		this.lift  = lift;
		this.state = state;

		addRequirements(lift);
	}

	@Override
	public void execute() {
		switch (state) {
			default:
				lift.floor();
			case LOW:
				lift.low();
			case MED:
				lift.med();
			case HIGH:
				lift.high();

		}
	}

	@Override
	public boolean isFinished() {
		return lift.atSetPoint();
	}

}
