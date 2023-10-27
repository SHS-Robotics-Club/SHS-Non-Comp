package org.firstinspires.ftc.teamcode.b_commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumRRSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

/**
 * A command to guide the robot to a parking zone based on the detected AprilTag.
 */
public class ParkCommand extends CommandBase {
    private static final int startX = 36, startY = 65, startH = 90;
    private final MecanumRRSubsystem drive;
    private final AprilTagSubsystem  tagSubsystem;
    StartingZone startingZone;

    /**
     * Constructs a ParkCommand.
     *
     * @param drive        The mecanum drive subsystem.
     * @param tagSubsystem The AprilTag detection subsystem.
     * @param startingZone The starting zone of the robot.
     */
    public ParkCommand(MecanumRRSubsystem drive, AprilTagSubsystem tagSubsystem, StartingZone startingZone) {
        this.drive        = drive;
        this.tagSubsystem = tagSubsystem;
        this.startingZone = startingZone;
    }

    @Override
    public void initialize() {
        // Get the trajectory for parking based on the starting pose and detected parking zone.
        TrajectorySequence park = getParkTrajectory(startingZone.getStartPose(), tagSubsystem.getParkingZone());
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

    /**
     * Generates the trajectory for parking based on the starting pose and detected parking zone.
     *
     * @param startPose The starting pose of the robot.
     * @param zone      The detected parking zone from AprilTag subsystem.
     * @return The trajectory sequence for parking.
     */
    public TrajectorySequence getParkTrajectory(Pose2d startPose, AprilTagSubsystem.ParkingZone zone) {
        // Set the initial pose of the robot.
        drive.setPoseEstimate(startPose);

        // Generate different trajectories based on the detected parking zone.
        switch (zone) {
            case LEFT:
                return drive.trajectorySequenceBuilder(startPose).forward(24).strafeLeft(24).build();
            case RIGHT:
                return drive.trajectorySequenceBuilder(startPose).forward(24).strafeRight(24).build();
            default:
                return drive.trajectorySequenceBuilder(startPose).forward(24).build();
        }
    }

    /**
     * Enum representing different starting zones for the robot.
     */
    public enum StartingZone {
        RED_LEFT(-startX, -startY, startH), RED_RIGHT(startX, -startY, startH), BLUE_LEFT(startX, startY, -startH), BLUE_RIGHT(-startX, startY, -startH);

        private final int X, Y, H;

        StartingZone(int X, int Y, int H) {
            this.X = X;
            this.Y = Y;
            this.H = H;
        }

        /**
         * Gets the starting pose for the specified zone.
         *
         * @return The starting pose of the robot.
         */
        public Pose2d getStartPose() {
            return new Pose2d(X, Y, Math.toRadians(H));
        }
    }
}
