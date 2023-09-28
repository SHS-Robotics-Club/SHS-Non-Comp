package org.firstinspires.ftc.teamcode.a_opmodes;

import static org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem.liftLevels;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.b_commands.ClawCommand;
import org.firstinspires.ftc.teamcode.b_commands.MecanumCommand;
import org.firstinspires.ftc.teamcode.c_subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.GamepadTrigger;

//@Disabled
@Config
@TeleOp(name = "MainTeleOp", group = ".Drive")
public class MainTeleOp extends CommandOpMode {
	Robot bot;

	@Override
	public void initialize() {
		//CommandScheduler.getInstance().reset();
		bot = new Robot(hardwareMap);
		telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

		GamepadEx gPad1 = new GamepadEx(gamepad1);

		MecanumCommand driveCommand = new MecanumCommand(bot.drive, gPad1::getLeftY, gPad1::getLeftX, gPad1::getRightX, liftLevels.getDriveMult());

		// CONTROLS ----------------------------------------------------------------------------------------------------
		// X Button = Claw Open/Close
/*		gPad1.getGamepadButton(GamepadKeys.Button.X)
		     .whenPressed(new ConditionalCommand(bot.CLAW_OPEN, bot.CLAW_CLOSE, () -> {
			         bot.claw.toggle();
			         return bot.claw.isOpen();
		         }));*/

		gPad1.getGamepadButton(GamepadKeys.Button.X)
		     .whenPressed(new ConditionalCommand(bot.CLAW_OPEN, bot.CLAW_CLOSE, () -> {
			     bot.claw.toggle();
			     return bot.claw.isOpen();
		     }));

		gPad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
		     .whenPressed(bot.LIFT_FLOOR);
		gPad1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
		     .whenPressed(bot.LIFT_LOW);
		gPad1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
		     .whenPressed(bot.LIFT_MED);
		gPad1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
		     .whenPressed(bot.LIFT_HIGH);

		gPad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
		     .whenPressed(bot.LOWER_T)
		     .whenReleased(bot.LOWER_F);

		new GamepadTrigger(gPad1, 0.25, GamepadKeys.Trigger.LEFT_TRIGGER).whileHeld(bot.LIFT_DOWN);
		new GamepadTrigger(gPad1, 0.25, GamepadKeys.Trigger.RIGHT_TRIGGER).whileHeld(bot.LIFT_UP);


		// Register and Schedule ----------------------------------------------------------------------------------------------------
		gPad1.readButtons();
		register(bot.lift);
		schedule(driveCommand.alongWith(new RunCommand(() -> {
			// Telemetry
			telemetry.update();
			telemetry.addData("Voltage", bot.voltageSensor.getVoltage());
			telemetry.addData("Lift Position", bot.lift.getPosition());
			telemetry.addData("Lift Velocity", bot.lift.getVelocity());
			telemetry.addData("Lift POS Error", bot.lift.getPositionError());
			telemetry.addData("Lift Modifier", bot.lift.getMod());
		})));

	}
}