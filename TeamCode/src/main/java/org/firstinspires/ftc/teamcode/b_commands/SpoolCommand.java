package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumRRSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.ServoSubsystem;

import java.util.function.DoubleSupplier;

public class SpoolCommand extends CommandBase {
    private final ServoSubsystem servo;
    private boolean finished;

    public SpoolCommand(ServoSubsystem servo, LiftSubsystem.Presets preset) {
        this.servo        = servo;
        addRequirements(servo);
    }

    @Override
    public boolean isFinished(){
        return finished;
    }
}