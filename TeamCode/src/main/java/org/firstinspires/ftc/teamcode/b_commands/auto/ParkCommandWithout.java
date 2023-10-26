package org.firstinspires.ftc.teamcode.b_commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

/**
 * A command to guide the robot to a parking zone based on the detected AprilTag without considering
 * the starting position. The starting pose is provided as a parameter.
 */
public class ParkCommandWithout extends CommandBase {
    private static final int startX = 36, startY = 65, startH = 90;
    private final MecanumSubsystem drive;
    private final AprilTagSubsystem tagSubsystem;
    private final Pose2d startPose;

    /**
     * Constructs a ParkCommandWithout.
     *
     * @param drive        The mecanum drive subsystem.
     * @param tagSubsystem The AprilTag detection subsystem.
     * @param startPose    The starting pose of the robot.
     */
    public ParkCommandWithout(MecanumSubsystem drive, AprilTagSubsystem tagSubsystem,
                              Pose2d startPose) {
        this.drive = drive;
        this.tagSubsystem = tagSubsystem;
        this.startPose = startPose;
    }

    @Override
    public void initialize() {
        // Get the trajectory for parking based on the provided starting pose and detected parking zone.
        TrajectorySequence park = getParkTrajectory(startPose, tagSubsystem.getParkingZone());
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
