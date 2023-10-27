package org.firstinspires.ftc.teamcode.a_opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.a_opmodes.Robot;
import org.firstinspires.ftc.teamcode.b_commands.auto.ParkCommandWithout;
import org.firstinspires.ftc.teamcode.b_commands.auto.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

/**
 * Autonomous OpMode named "Pl: Left". This OpMode instructs the robot to perform a preloaded cone maneuver
 * from the left position.
 */
@Config
@Autonomous(name = "Pl: Left", group = ".Score", preselectTeleOp = "MainTeleOp")
public class PreloadLeft extends CommandOpMode {

    @Override
    public void initialize() {
        // Initialize robot hardware
        final Robot bot = new Robot(hardwareMap, true);

        // Setup telemetry for dashboard
        FtcDashboard dashboard          = FtcDashboard.getInstance(); // FTC Dashboard Instance
        Telemetry    dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set up trajectory constraints for the preloaded ring maneuver
        TrajectoryVelocityConstraint preload = new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(30), new AngularVelocityConstraint(5)));

        // Define the trajectory sequences for the autonomous maneuver
        TrajectorySequence auto1 = bot.drive.trajectorySequenceBuilder(new Pose2d(-40.00, -64.00, Math.toRadians(90.00))).splineTo(new Vector2d(-34.00, -42.00), Math.toRadians(90.00)).setVelConstraint(preload).splineTo(new Vector2d(-27.00, -9.50), Math.toRadians(65.00)).build();

        TrajectorySequence auto2 = bot.drive.trajectorySequenceBuilder(auto1.end()).setReversed(true).splineTo(new Vector2d(-35.00, -37.00), Math.toRadians(-90.00)).build();

        // Set robot's initial pose to the start of the first trajectory sequence
        bot.drive.setPoseEstimate(auto1.start());

        // Wait for the start command from the driver station
        waitForStart();

        // Register the robot lift subsystem
        register(bot.lift);

        // Start camera stream on the dashboard
        schedule(new RunCommand(() -> {
            if (isStopRequested()) return;
            dashboard.startCameraStream(bot.aprilTag.getCamera(), 0);
            telemetry.update();
        }));

        // Execute a sequence of commands for the preloaded ring maneuver
        schedule(new SequentialCommandGroup(bot.DETECTOR_WAIT.withTimeout(2000), bot.CLAW_CLOSE, new WaitCommand(1000), new TrajectoryFollowerCommand(bot.drive, auto1).alongWith(new WaitCommand(500).andThen(bot.LIFT_HIGH)), bot.CLAW_OPEN, new WaitCommand(1000), bot.LIFT_GROUND.alongWith(new TrajectoryFollowerCommand(bot.drive, auto2)), new ParkCommandWithout(bot.drive, bot.aprilTag, auto2.end())));
    }
}