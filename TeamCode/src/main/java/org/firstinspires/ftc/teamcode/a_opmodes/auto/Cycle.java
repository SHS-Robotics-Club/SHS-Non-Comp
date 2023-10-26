package org.firstinspires.ftc.teamcode.a_opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.a_opmodes.Robot;
import org.firstinspires.ftc.teamcode.d_roadrunner.trajectorysequence.TrajectorySequence;

/**
 * Autonomous OpMode named "Cycle". This OpMode performs a predefined sequence of movements for
 * the robot to complete a specific task.
 */
@Autonomous(name = "Cycle", group = ".Score", preselectTeleOp = "MainTeleOp")
public class Cycle extends CommandOpMode {
    @Override
    public void initialize() {
        // Initialize robot hardware
        final Robot bot = new Robot(hardwareMap, true);

        // Setup telemetry for dashboard
        FtcDashboard dashboard          = FtcDashboard.getInstance(); // FTC Dashboard Instance
        Telemetry    dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set the initial robot pose
        bot.drive.setPoseEstimate(new Pose2d(-40.00, -64.00, Math.toRadians(90.00)));

        // Define the autonomous trajectory for the robot
        TrajectorySequence auto1 = bot.drive.trajectorySequenceBuilder(new Pose2d(-40.00, -64.00, Math.toRadians(90.00))).splineTo(new Vector2d(-34.00, -42.00), Math.toRadians(90.00)).splineTo(new Vector2d(-27.00, -9.50), Math.toRadians(55.00)).build();

        // Wait for the start command from the driver station
        waitForStart();

        // Register robot subsystems and commands
        register(bot.lift);

        // Run commands sequentially for autonomous movement
        schedule(new RunCommand(() -> {
            if (isStopRequested()) return;
            // Start camera stream on the dashboard
            dashboard.startCameraStream(bot.aprilTag.getCamera(), 0);
            telemetry.update();
        }));

        // Schedule commands for robot actions
        schedule(bot.CLAW_CLOSE.andThen(new WaitCommand(500)).andThen(bot.LIFT_HIGH).andThen(new WaitCommand(5000)).andThen(bot.LIFT_FLOOR));
    }
}