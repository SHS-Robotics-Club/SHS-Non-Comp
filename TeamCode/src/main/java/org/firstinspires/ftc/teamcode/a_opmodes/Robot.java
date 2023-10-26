package org.firstinspires.ftc.teamcode.a_opmodes;

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
import org.firstinspires.ftc.teamcode.c_subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagSubsystem;
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
    public static double akP = 0.005, akI = 0.0, akD = 0, akF = 0.0;

    // Hardware components
    public VoltageSensor voltageSensor;
    public MotorEx       liftLeft, liftRight;
    public MotorGroup liftGroup;
    public ServoEx    clawLeft, clawRight;
    public CRServo          spool;
    public FtcDashboard     dashboard = FtcDashboard.getInstance();
    public List<LynxModule> revHubs;

    // Subsystems
    public MecanumSubsystem  drive;
    public ClawSubsystem     claw;
    public LiftSubsystem     lift;
    public AprilTagSubsystem aprilTag;

    // Commands
    public InstantCommand CLAW_TOGGLE, CLAW_OPEN, CLAW_CLOSE, LIFT_FLOOR, LIFT_LOW, LIFT_MED, LIFT_HIGH, LIFT_LOWER, LIFT_DELOWER, LIFT_DOWN, LIFT_UP;
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
        // Initialize sensors and dashboard
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        dashboard     = FtcDashboard.getInstance();

        // Bulk Read for REV Hubs
        revHubs = hardwareMap.getAll(LynxModule.class);
        //revHubs.get(1).getCurrent("volt");
        for (LynxModule hub : revHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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
        drive = new MecanumSubsystem(new MecanumDrive(hardwareMap), true);
        claw  = new ClawSubsystem(clawLeft, clawRight);
        lift  = new LiftSubsystem(liftGroup, spool);

        // Initialize commands
        CLAW_OPEN   = new InstantCommand(claw::openClaw, claw);
        CLAW_CLOSE  = new InstantCommand(claw::closeClaw, claw);
        CLAW_TOGGLE = new InstantCommand(claw::toggleClaw, claw);

        LIFT_FLOOR = new InstantCommand(lift::moveToFloor, lift);
        LIFT_LOW   = new InstantCommand(lift::moveToLow, lift);
        LIFT_MED   = new InstantCommand(lift::moveToMedium, lift);
        LIFT_HIGH  = new InstantCommand(lift::moveToHigh, lift);

        LIFT_LOWER   = new InstantCommand(() -> lift.lower(true));
        LIFT_DELOWER = new InstantCommand(() -> lift.lower(false));

        LIFT_DOWN = new InstantCommand(() -> lift.decreaseModifier(15));
        LIFT_UP   = new InstantCommand(() -> lift.increaseModifier(15));

        if (isAuto) {
            autoConfig(hardwareMap);
        }

    }

    private void autoConfig(HardwareMap hardwareMap) {

        lift.setCoefficients(akP, akI, akD, akF);

        aprilTag = new AprilTagSubsystem(hardwareMap, "Webcam 1", 1280, 720, 0.4, 1552.74274588, 1552.74274588, 793.573231003, 202.006088244);
        aprilTag.init();

        DETECTOR_WAIT = new WaitUntilCommand(aprilTag::foundZone);
    }
}