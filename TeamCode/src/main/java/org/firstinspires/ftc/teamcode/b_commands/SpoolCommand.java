package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.ServoSubsystem;

public class SpoolCommand extends CommandBase {
    private final ServoSubsystem servo;
    private final LiftSubsystem  lift;
    private       boolean        finished;
    private final ElapsedTime runtime = new ElapsedTime();
    private static final double SPOOL_TIMEOUT = 5.0; // seconds
    private static final double SERVO_POWER = 1.0;

    public SpoolCommand(ServoSubsystem servo, LiftSubsystem lift) {
        this.servo = servo;
        this.lift  = lift;
        addRequirements(servo);
    }

    @Override
    public void initialize() {
        runtime.reset();
    }

    @Override
    public void execute() {
        if (lift.getPosition() < LiftSubsystem.Presets.LOW.getPresetHeight() && runtime.seconds() > SPOOL_TIMEOUT){
            servo.setPower(SERVO_POWER);
        } else {
            finished = true;
        }
    }

    public void end() {
        servo.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}