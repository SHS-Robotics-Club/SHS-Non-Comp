package org.firstinspires.ftc.teamcode.a_opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.b_commands.MecanumCommand;
import org.firstinspires.ftc.teamcode.c_subsystems.GamepadTrigger;

//@Disabled
@Config
@TeleOp(name = "MainTeleOp", group = ".Drive")
public class MainTeleOp extends CommandOpMode {
    Robot robot;

    @Override
    public void initialize() {
        // Initialize the robot hardware
        robot = new Robot(hardwareMap);

        // Set up telemetry on both Driver Station and Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Set up gamepad controls
        GamepadEx gamepad1Ex = new GamepadEx(gamepad1);

        // Create a MecanumCommand for driving the robot
        MecanumCommand driveCommand = new MecanumCommand(robot.drive, gamepad1Ex::getLeftY, gamepad1Ex::getLeftX, gamepad1Ex::getRightX, robot.lift.getDriveMultiplier());

        // Define gamepad button commands
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.X).whenPressed(robot.CLAW_TOGGLE);
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(robot.LIFT_FLOOR);
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(robot.LIFT_LOW);
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(robot.LIFT_MED);
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(robot.LIFT_HIGH);
        gamepad1Ex.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(robot.LIFT_LOWER).whenReleased(robot.LIFT_DELOWER);

        // Create gamepad triggers for controlling lift movement
        new GamepadTrigger(gamepad1Ex, 0.25, GamepadKeys.Trigger.LEFT_TRIGGER).whileHeld(robot.LIFT_DOWN);
        new GamepadTrigger(gamepad1Ex, 0.25, GamepadKeys.Trigger.RIGHT_TRIGGER).whileHeld(robot.LIFT_UP);


        // Register and schedule commands
        gamepad1Ex.readButtons();
        register(robot.lift);
        schedule(driveCommand.alongWith(new RunCommand(() -> {
            // Update telemetry data
            telemetry.update();
            telemetry.addData("Voltage", robot.voltageSensor.getVoltage());
            telemetry.addData("Lift Position", robot.lift.getPosition());
            telemetry.addData("Lift Velocity", robot.lift.getVelocity());
            telemetry.addData("Lift POS Error", robot.lift.getPositionError());
            telemetry.addData("Lift Modifier", robot.lift.getModifier());
        })));
    }
}