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

@Config
@Autonomous(name = "Pl: Right", group = ".Score", preselectTeleOp = "MainTeleOp")
public class PreloadRight extends CommandOpMode {
	@Override
	public void initialize() {
		// Get Devices
		final Robot bot = new Robot(hardwareMap, true);

		// Setup Telemetry
		FtcDashboard dashboard          = FtcDashboard.getInstance(); //FTC Dashboard Instance
		Telemetry    dashboardTelemetry = dashboard.getTelemetry();
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		TrajectoryVelocityConstraint preload = new MinVelocityConstraint(Arrays.asList(new TranslationalVelocityConstraint(30), new AngularVelocityConstraint(5)));

		TrajectorySequence auto1 = bot.drive.trajectorySequenceBuilder(new Pose2d(30.00, -64.00, Math.toRadians(90.00)))
		                                    .splineTo(new Vector2d(34.00, -42.00), Math.toRadians(90.00))
		                                    .setVelConstraint(preload)
		                                    .splineTo(new Vector2d(28.00, -8.50), Math.toRadians(135))
		                                    .build();

		TrajectorySequence auto2 = bot.drive.trajectorySequenceBuilder(auto1.end())
		                                    .setReversed(true)
		                                    .splineTo(new Vector2d(35.00, -37.00), Math.toRadians(-90.00))
		                                    .build();

		bot.drive.setPoseEstimate(auto1.start());

		waitForStart();

		register(bot.lift);
		schedule(new RunCommand(() -> {
			if (isStopRequested()) return;
			dashboard.startCameraStream(bot.aprilTag.getCamera(), 0);
			telemetry.update();
		}));

		schedule(new SequentialCommandGroup(
				         bot.DETECTOR_WAIT.withTimeout(2000),
				         bot.CLAW_CLOSE,
				         new WaitCommand(1000),
				         new TrajectoryFollowerCommand(bot.drive, auto1).alongWith(new WaitCommand(500).andThen(bot.LIFT_HIGH)),
				         bot.CLAW_OPEN,
				         new WaitCommand(1000),
				         bot.LIFT_FLOOR.alongWith(new TrajectoryFollowerCommand(bot.drive, auto2)),
						 new WaitCommand(5000),
				         new ParkCommandWithout(bot.drive, bot.aprilTag, auto2.end()))
		        );


/*		bot.DETECTOR_WAIT.withTimeout(5000)
		                 .andThen(bot.CLAW_CLOSE)
		                 .alongWith(new WaitCommand(1000))
		                 .andThen(bot.LIFT_HIGH)
		                 .alongWith(new TrajectoryFollowerCommand(bot.drive, auto1))
		                 .andThen(bot.CLAW_OPEN)
		                 .andThen(new WaitCommand(5000))
		                 .andThen(bot.LIFT_FLOOR)
		                 .alongWith(new TrajectoryFollowerCommand(bot.drive, auto2))
		                 .andThen(new ParkCommandWithout(bot.drive, bot.aprilTag, ParkCommandWithout.StartingZone.BLUE_LEFT))*/
	}
}