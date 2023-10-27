package org.firstinspires.ftc.teamcode.b_commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumRRSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

/**
 * A command to follow a trajectory sequence using a mecanum drive subsystem.
 */
public class TrajectoryFollowerCommand extends CommandBase {

    private final MecanumRRSubsystem drive;
    private final TrajectorySequence trajectory;

    /**
     * Constructs a TrajectoryFollowerCommand.
     *
     * @param drive      The mecanum drive subsystem.
     * @param trajectory The trajectory sequence to follow.
     */
    public TrajectoryFollowerCommand(MecanumRRSubsystem drive, TrajectorySequence trajectory) {
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
