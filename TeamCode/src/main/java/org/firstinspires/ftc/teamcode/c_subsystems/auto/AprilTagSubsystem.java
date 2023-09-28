package org.firstinspires.ftc.teamcode.c_subsystems.auto;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


public class AprilTagSubsystem extends SubsystemBase {
	public static ParkingZone      lastParkingZone = ParkingZone.NONE;
	private final HardwareMap      hardwareMap;
	private final int              WIDTH;
	private final int              HEIGHT;
	private final Object           sync            = new Object();
	public        Status           status          = Status.NOT_CONFIGURED;
	private       OpenCvCamera     camera;
	private       String           cameraName;
	private       AprilTagPipeline aprilTagPipeline;
	private       double           tagSize; // Metres
	private       double           fx, fy, cx, cy;// Pixels

	public AprilTagSubsystem(HardwareMap hardwareMap, String cameraName, int width, int height,
	                         double tagSize, double fx, double fy, double cx, double cy) {
		this.hardwareMap = hardwareMap;
		this.cameraName  = cameraName;
		WIDTH            = width;
		HEIGHT           = height;

		this.tagSize = tagSize;
		this.fx      = fx;
		this.fy      = fy;
		this.cx      = cx;
		this.cy      = cy;
	}

	public Status getStatus() {
		synchronized (sync) {
			return status;
		}
	}

	public void init() {
		lastParkingZone = ParkingZone.NONE;
		synchronized (sync) {
			if (status == Status.NOT_CONFIGURED) {
				int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
						"cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

				camera = OpenCvCameraFactory.getInstance().createWebcam(
						hardwareMap.get(WebcamName.class, cameraName), cameraMonitorViewId);

				camera.setPipeline(
						aprilTagPipeline = new AprilTagPipeline(tagSize, fx, fy, cx, cy));

				status = Status.INITIALIZING;

				camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
					@Override
					public void onOpened() {
						camera.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);

						synchronized (sync) {
							status = Status.RUNNING;
						}
					}

					@Override
					public void onError(int errorCode) {
						synchronized (sync) {
							status = Status.FAILURE;
						}

						RobotLog.addGlobalWarningMessage(
								"Warning: Camera device failed to open with EasyOpenCv error: " +
								((errorCode == -1) ?
								 "CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE" :
								 "CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE")); //Warn the user about the issue
					}
				});
			}
		}
	}

	public OpenCvCamera getCamera() {
		return camera;
	}

	public int getZoneId() {
		ArrayList<AprilTagDetection> currentDetections = aprilTagPipeline.getLatestDetections();
		AprilTagDetection            tagOfInterest     = null;
		boolean                      tagFound          = false;

		if (currentDetections.size() != 0) {
			for (AprilTagDetection tag : currentDetections) {
				if (tag.id >= 1 && tag.id <= 3) {
					tagOfInterest = tag;
					tagFound      = true;
					break;
				}
			}
		}

		if (tagFound) {
			return tagOfInterest.id;
		} else {
			return -1;
		}
	}

	public ParkingZone getParkingZone() {
		if (lastParkingZone == ParkingZone.NONE) {
			switch (getZoneId()) {
				case 1:
					return ParkingZone.LEFT;
				case 2:
					return ParkingZone.CENTER;
				case 3:
					return ParkingZone.RIGHT;
				default:
					return ParkingZone.NONE;
			}
		} else {
			return lastParkingZone;
		}

	}

	public boolean foundZone() {
		if (getParkingZone() == ParkingZone.NONE) {
			return false;
		} else {
			lastParkingZone = getParkingZone();
			status          = AprilTagSubsystem.Status.STOPPED;
			return true;
		}
	}

	public enum Status {
		NOT_CONFIGURED, INITIALIZING, RUNNING, FAILURE, STOPPED
	}

	public enum ParkingZone {
		LEFT, CENTER, RIGHT, NONE
	}
}