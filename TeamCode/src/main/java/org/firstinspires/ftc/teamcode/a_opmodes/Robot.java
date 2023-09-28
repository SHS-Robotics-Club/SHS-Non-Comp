package org.firstinspires.ftc.teamcode.a_opmodes;

/*
 * TYPE			NAME			ID		    DESCRIPTION
 * ------------------------------------------------------------
 * MOTOR		liftLeft		liftL		Lift Motor Left
 * MOTOR		liftRight		liftR		Lift Motor Right
 *
 * SERVO        clawLeft        clawLeft    Claw Left (Open/Close)
 * SERVO        clawRight       clawRight   Claw Right (Open/Close)
 *
 * CRSERVO		spool			spool		Tensions MGN Rail
 */

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
import org.firstinspires.ftc.teamcode.b_commands.ClawCommand;
import org.firstinspires.ftc.teamcode.c_subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.c_subsystems.auto.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.d_roadrunner.drive.MecanumDrive;

import java.util.List;

@Config
public class Robot {
	public static double akP = 0.005, akI = 0.0, akD = 0, akF = 0.0;
	public VoltageSensor voltageSensor;

	public MotorEx liftLeft, liftRight;// Motors
	public MotorGroup liftGroup;// Motor Group
	public ServoEx    clawLeft, clawRight; // Servos
	public CRServo spool; // CR Servo
	boolean isAuto;

	// MISC DEFINITIONS
	public FtcDashboard     dashboard = FtcDashboard.getInstance(); //FTC Dashboard Instance
	public List<LynxModule> revHubs; //Lynx Module for REV Hubs

	public MecanumSubsystem drive;
	public ClawSubsystem    claw;
	public LiftSubsystem    lift;

	public InstantCommand LIFT_FLOOR, LIFT_LOW, LIFT_MED, LIFT_HIGH, LOWER_T, LOWER_F, LIFT_DOWN, LIFT_UP;
	public ClawCommand CLAW_OPEN, CLAW_CLOSE;
	//public LiftCommand LIFT_FLOOR, LIFT_LOW, LIFT_MED, LIFT_HIGH;

	public AprilTagSubsystem aprilTag;
	public WaitUntilCommand  DETECTOR_WAIT;

	public Robot(HardwareMap hardwareMap) {
		this(hardwareMap, false);
	}

	public Robot(HardwareMap hardwareMap, boolean isAuto) {

		voltageSensor = hardwareMap.voltageSensor.iterator().next();

		this.isAuto = isAuto;

		// Bulk Read
		revHubs = hardwareMap.getAll(LynxModule.class);

		for (LynxModule hub : revHubs) {
			hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
		}

		// MOTORS ----------------------------------------------------------------------------------------------------
		// Map
		liftLeft  = new MotorEx(hardwareMap, "liftL", MotorEx.GoBILDA.RPM_312);
		liftRight = new MotorEx(hardwareMap, "encoderLeftLift", MotorEx.GoBILDA.RPM_312);

		// Reset encoders and set direction
		liftLeft.setInverted(true);
		liftLeft.resetEncoder();

		// Group lift motors
		liftGroup = new MotorGroup(liftLeft, liftRight);

		// Set RunMode for motors (RawPower, VelocityControl, PositionControl)
		liftGroup.setRunMode(MotorEx.RunMode.VelocityControl);

		// Break
		liftGroup.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);

		// SERVOS ----------------------------------------------------------------------------------------------------
		// Map
		clawLeft  = new SimpleServo(hardwareMap, "clawLeft", -180, 180, AngleUnit.DEGREES);
		clawRight = new SimpleServo(hardwareMap, "clawRight", -180, 180, AngleUnit.DEGREES);
		spool     = new CRServo(hardwareMap, "spool");

		// Invert
		clawRight.setInverted(true);
		spool.setInverted(true);

		// Default POS
/*		clawLeft.turnToAngle(30);
		clawRight.turnToAngle(30);*/

		// COMMANDS & SUBSYSTEMS --------------------------------------------------------------------------------------
		drive = new MecanumSubsystem(new MecanumDrive(hardwareMap), true);
		claw  = new ClawSubsystem(clawLeft, clawRight);
		lift  = new LiftSubsystem(liftGroup, spool);

/*		CLAW_OPEN  = new InstantCommand(claw::open, claw);
		CLAW_CLOSE = new InstantCommand(claw::close, claw);*/

		CLAW_OPEN  = new ClawCommand(claw, ClawSubsystem.ClawState.OPEN);
		CLAW_CLOSE = new ClawCommand(claw, ClawSubsystem.ClawState.CLOSE);

/*		CLAW_TOGGLE = new ConditionalCommand(CLAW_OPEN, CLAW_CLOSE, () -> {
			claw.toggle();
			return claw.isOpen();
		});*/

		LIFT_FLOOR = new InstantCommand(lift::floor, lift);
		LIFT_LOW   = new InstantCommand(lift::low, lift);
		LIFT_MED   = new InstantCommand(lift::med, lift);
		LIFT_HIGH  = new InstantCommand(lift::high, lift);

/*		LIFT_FLOOR = new LiftCommand(lift, LiftSubsystem.LiftLevels.FLOOR);
		LIFT_LOW   = new LiftCommand(lift, LiftSubsystem.LiftLevels.LOW);
		LIFT_MED   = new LiftCommand(lift, LiftSubsystem.LiftLevels.MED);
		LIFT_HIGH  = new LiftCommand(lift, LiftSubsystem.LiftLevels.HIGH);*/

		LOWER_T = new InstantCommand(() -> lift.lower(true));
		LOWER_F = new InstantCommand(() -> lift.lower(false));

		LIFT_DOWN = new InstantCommand(() -> lift.down(15));
		LIFT_UP   = new InstantCommand(() -> lift.up(15));

		if (isAuto) {
			autoConfig(hardwareMap);
		}

	}

	private void autoConfig(HardwareMap hardwareMap) {

		lift.setCoefficients(akP, akI, akD, akF);

		aprilTag = new AprilTagSubsystem(hardwareMap, "Webcam 1", 1280, 720, 0.4, 1552.74274588,
		                                 1552.74274588, 793.573231003, 202.006088244);
		aprilTag.init();

		DETECTOR_WAIT = new WaitUntilCommand(aprilTag::foundZone);
	}
}