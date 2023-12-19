package org.firstinspires.ftc.teamcode.c_subsystems.auto;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.List;

/**
 * Subsystem for AprilTag detection and parking zone identification.
 */
public class AprilTagVPSubsystem extends SubsystemBase {

    // Constants
    public static ParkingZone       lastParkingZone = ParkingZone.NONE;
    private final HardwareMap       hardwareMap;
    private final int               WIDTH;
    private final int               HEIGHT;
    private final String            cameraName;
    private final double            tagSize; // Metres
    private final double            fx;
    private final double            fy;
    private final double            cx;
    private final double            cy;// Pixels
    private       AprilTagProcessor aprilTag;
    private       VisionPortal      visionPortal;
    // Camera properties
    private       OpenCvCamera      camera;

    /**
     * Constructor for AprilTagSubsystem.
     *
     * @param hardwareMap Hardware map for accessing robot components.
     * @param cameraName  Name of the webcam.
     * @param width       Width of the camera frame.
     * @param height      Height of the camera frame.
     * @param tagSize     Size of the AprilTag in meters.
     * @param fx          X focal length.
     * @param fy          Y focal length.
     * @param cx          X center pixel coordinate.
     * @param cy          Y center pixel coordinate.
     */
    public AprilTagVPSubsystem(HardwareMap hardwareMap, String cameraName, int width, int height,
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

    public void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(false).setDrawCubeProjection(false).setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS).setLensIntrinsics(fx,
                                                                                      fy,
                                                                                      cx,
                                                                                      cy)

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                .setCameraResolution(new Size(WIDTH, HEIGHT)).enableLiveView(true)
                //.setStreamFormat(VisionPortal.StreamFormat.YUY2); // Set the stream format; MJPEG uses less bandwidth than default YUY2.
                .setAutoStopLiveView(false).addProcessor(aprilTag)
                .build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    /**
     * Get the ID of the detected zone.
     *
     * @return Detected zone ID.
     */
    public int getZoneId() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        AprilTagDetection       tagOfInterest     = null;
        boolean                 tagFound          = false;

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

    /**
     * Get the identified parking zone.
     *
     * @return Identified parking zone.
     */
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

    /**
     * Check if a parking zone is detected.
     *
     * @return True if a zone is detected, false otherwise.
     */
    public boolean foundZone() {
        if (getParkingZone() == ParkingZone.NONE) {
            return false;
        } else {
            lastParkingZone = getParkingZone();
            return true;
        }
    }

    /**
     * Enumeration representing different parking zones.
     */
    public enum ParkingZone {
        LEFT, CENTER, RIGHT, NONE
    }
}