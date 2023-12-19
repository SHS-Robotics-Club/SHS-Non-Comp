package org.firstinspires.ftc.teamcode.a_opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.c_subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumRRSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagVPSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.drive.MecanumDrive;

import java.util.List;

/*
 * TYPE          NAME            ID          DESCRIPTION
 * ------------------------------------------------------------
 * MOTOR         liftLeft        liftL       Lift Motor Left
 * MOTOR         liftRight       liftR       Lift Motor Right
 *
 * SERVO         clawLeft        clawLeft    Claw Left (Open/Close)
 * SERVO         clawRight       clawRight   Claw Right (Open/Close)
 *
 * CRSERVO       spool           spool       Tensions MGN Rail
 */

/**
 * Represents the robot's hardware components and subsystems.
 */
@Config
public class Robot {
    // Constants for PID coefficients
    public static double PID_COEFFICIENT_P = 0.005;
    public static double PID_COEFFICIENT_I = 0.0;
    public static double PID_COEFFICIENT_D = 0;
    public static double PID_COEFFICIENT_F = 0.0;

    // Hardware components
    public VoltageSensor voltageSensor;
    public MotorEx       liftLeft, liftRight;
    public MotorGroup liftGroup;
    public ServoEx    clawLeft, clawRight;
    public CRServo          spool;
    public FtcDashboard     dashboard = FtcDashboard.getInstance();
    public List<LynxModule> revHubs;

    // Subsystems
    public MecanumRRSubsystem drive;
    public ClawSubsystem      claw;
    public LiftSubsystem       lift;
    public AprilTagVPSubsystem aprilTag;

    // Commands
    public InstantCommand BOT_180, CLAW_TOGGLE, CLAW_OPEN, CLAW_CLOSE, LIFT_GROUND, LIFT_LOW, LIFT_MED, LIFT_HIGH, LIFT_LOWER, LIFT_DELOWER, LIFT_DOWN, LIFT_UP;
    public WaitUntilCommand DETECTOR_WAIT;


    /**
     * Constructor for the Robot class.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     */
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    /**
     * Overloaded constructor for the Robot class.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     * @param isAuto      A flag indicating if the robot is in autonomous mode.
     */
    public Robot(HardwareMap hardwareMap, boolean isAuto) {
        configureRobot(hardwareMap, isAuto);

        // Initialize sensors and dashboard
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        dashboard     = FtcDashboard.getInstance();

        // Bulk Read for REV Hubs
        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : revHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (this.getVoltage(VoltageUnit.VOLTS) < 12) {
            gamepad1.rumbleBlips(5);
            gamepad1.setLedColor(255, 0, 0, 120000);
        }
    }

    /**
     * Configures the robot's hardware components and subsystems.
     *
     * @param hardwareMap The hardware map for accessing robot components.
     * @param isAuto      A flag indicating if the robot is in autonomous mode.
     */
    private void configureRobot(HardwareMap hardwareMap, boolean isAuto) {
        // Initialize motors and servos
        liftLeft  = new MotorEx(hardwareMap, "liftL", MotorEx.GoBILDA.RPM_312);
        liftRight = new MotorEx(hardwareMap, "encoderLeftLift", MotorEx.GoBILDA.RPM_312);
        liftLeft.setInverted(true);
        liftLeft.resetEncoder();
        liftGroup = new MotorGroup(liftLeft, liftRight);
        liftGroup.setRunMode(MotorEx.RunMode.VelocityControl);
        liftGroup.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

        clawLeft  = new SimpleServo(hardwareMap, "clawLeft", -180, 180, AngleUnit.DEGREES);
        clawRight = new SimpleServo(hardwareMap, "clawRight", -180, 180, AngleUnit.DEGREES);
        clawRight.setInverted(true);

        spool = new CRServo(hardwareMap, "spool");
        spool.setInverted(true);

        // Initialize subsystems
        drive = new MecanumRRSubsystem(new MecanumDrive(hardwareMap), true);
        claw  = new ClawSubsystem(clawLeft, clawRight);
        lift  = new LiftSubsystem(liftGroup);

        // Initialize commands
        CLAW_OPEN   = new InstantCommand(claw::openClaw, claw);
        CLAW_CLOSE  = new InstantCommand(claw::closeClaw, claw);
        CLAW_TOGGLE = new InstantCommand(claw::toggleClaw, claw);

        LIFT_GROUND = new InstantCommand(() -> lift.moveToPreset(LiftSubsystem.Presets.GROUND));
        LIFT_LOW    = new InstantCommand(() -> lift.moveToPreset(LiftSubsystem.Presets.LOW));
        LIFT_MED    = new InstantCommand(() -> lift.moveToPreset(LiftSubsystem.Presets.MEDIUM));
        LIFT_HIGH   = new InstantCommand(() -> lift.moveToPreset(LiftSubsystem.Presets.HIGH));

        LIFT_LOWER   = new InstantCommand(() -> lift.modifyPosition(-200));
        LIFT_DELOWER = new InstantCommand(() -> lift.modifyPosition(200));

        LIFT_DOWN = new InstantCommand(() -> lift.modifyPosition(-15));
        LIFT_UP   = new InstantCommand(() -> lift.modifyPosition(15));

        BOT_180 = new InstantCommand(() -> drive.turn180());



        if (isAuto) {
            // Configure auto-specific components
            lift.setCoefficients(PID_COEFFICIENT_P,
                                 PID_COEFFICIENT_I,
                                 PID_COEFFICIENT_D,
                                 PID_COEFFICIENT_F);
            aprilTag = new AprilTagVPSubsystem(hardwareMap,
                                             "Webcam 1",
                                             1280,
                                             720,
                                             0.4,
                                             1552.74274588,
                                             1552.74274588,
                                             793.573231003,
                                             202.006088244);
            aprilTag.initAprilTag();
            DETECTOR_WAIT = new WaitUntilCommand(aprilTag::foundZone);
        }
    }

    /**
     * Retrieves and returns the current voltage of the robot.
     *
     * @param unit The unit in which the voltage is measured (e.g., VoltageUnit.VOLTS).
     * @return The current voltage of the robot in the specified unit.
     */
    public double getVoltage(VoltageUnit unit) {
        return revHubs.get(0).getInputVoltage(unit);
    }

    /**
     * Calculates and returns the total current draw of the robot.
     *
     * @param unit The unit in which the current is measured (e.g., CurrentUnit.AMPS).
     * @return The total current draw of the robot in the specified unit.
     */
    public double getCurrent(CurrentUnit unit) {
        double totalCurrent = 0;
        for (LynxModule hub : revHubs) {
            totalCurrent += hub.getCurrent(unit);
        }
        return totalCurrent;
    }
}