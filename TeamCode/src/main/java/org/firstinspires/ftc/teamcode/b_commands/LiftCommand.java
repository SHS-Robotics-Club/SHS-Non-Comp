package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumRRSubsystem;

import java.util.function.DoubleSupplier;

public class LiftCommand extends CommandBase {
    private final LiftSubsystem  lift;

    public LiftCommand(LiftSubsystem lift, LiftSubsystem.Presets preset) {
        this.lift        = lift;
        lift.moveToPreset(preset);
        addRequirements(lift);
    }

    public LiftCommand(LiftSubsystem lift, int position) {
        this.lift        = lift;
        lift.setSetpoint(position);
        addRequirements(lift);
    }

    @Override
    public boolean isFinished(){
        return lift.atSetpoint();
    }
}