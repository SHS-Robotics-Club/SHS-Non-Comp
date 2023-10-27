package org.firstinspires.ftc.teamcode.c_subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;

@Config
public class ServoSubsystem extends SubsystemBase {
    private final CRServo servo;

    public ServoSubsystem(CRServo servo) {
        this.servo  = servo;
    }

    public void setPower(double output) {
        servo.setPower(output);
    }
}
