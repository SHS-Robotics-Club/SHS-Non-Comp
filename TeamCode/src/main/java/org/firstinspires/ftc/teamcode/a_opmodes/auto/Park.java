package org.firstinspires.ftc.teamcode.a_opmodes.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.a_opmodes.Robot;
import org.firstinspires.ftc.teamcode.b_commands.auto.ParkCommand;

/**
 * Autonomous OpMode named "ParkOnly". This OpMode instructs the robot to perform parking maneuvers.
 */
@Autonomous(name = "ParkOnly", group = ".Park", preselectTeleOp = "MainTeleOp")
public class Park extends CommandOpMode {
    @Override
    public void initialize() {
        // Initialize robot hardware
        final Robot bot = new Robot(hardwareMap, true);

        // Setup telemetry for dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance(); // FTC Dashboard Instance
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Start camera stream on the dashboard
        schedule(new RunCommand(() -> {
            if (isStopRequested()) return;
            dashboard.startCameraStream(bot.aprilTag.getCamera(), 0);
            telemetry.update();
        }));

        // Wait for the start command from the driver station
        waitForStart();

        // Execute a sequence of commands for parking the robot
        schedule(new SequentialCommandGroup(bot.CLAW_CLOSE, bot.DETECTOR_WAIT.withTimeout(20000), new ParkCommand(bot.drive, bot.aprilTag, ParkCommand.StartingZone.BLUE_LEFT)));
    }
}