package org.firstinspires.ftc.teamcode.b_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;

public class LiftManualCommand extends CommandBase {
    private final LiftSubsystem lift;
    private int increment;
    private boolean held;

    public LiftManualCommand(LiftSubsystem lift, int increment, boolean held) {
        this.lift      = lift;
        this.increment = increment;
        this.held = held;
        addRequirements(lift);
    }

    @Override
    public void execute() {
        // Perform the height adjustment logic using LiftControl interface methods
        lift.modifyPosition(increment);
        // Or use m_liftControl.decrementHeight(m_decrementAmount); for decrementing
    }

    @Override
    public boolean isFinished(){
        return held;
    }
}
