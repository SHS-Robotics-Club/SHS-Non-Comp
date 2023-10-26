package org.firstinspires.ftc.teamcode.c_subsystems.auto;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

/**
 * Pipeline for detecting AprilTags and extracting pose information.
 */
public class AprilTagPipeline extends OpenCvPipeline {
    private final Object                       detectionsUpdateSync = new Object();
    private final Object                       decimationSync       = new Object();
    private       Mat    cameraMatrix;
    private final Scalar blue = new Scalar(7, 197, 235, 255);
    private final Scalar red  = new Scalar(255, 0, 0, 255);
    private final Scalar green = new Scalar(0, 255, 0, 255);
    private final Scalar white = new Scalar(255, 255, 255, 255);
    private final double fx;
    private final double fy;
    private final double cx;
    private final double cy;
    // UNITS ARE METERS
    private final double tagSize;
    private final double tagSizeX;
    private final double tagSizeY;
    private       long   nativeApriltagPtr;
    private final Mat                          grey       = new Mat();
    private       ArrayList<AprilTagDetection> detections = new ArrayList<>();
    private       ArrayList<AprilTagDetection> detectionsUpdate     = new ArrayList<>();
    private       float                        decimation;
    private       boolean                      needToSetDecimation;

    /**
     * Constructor for AprilTagPipeline.
     *
     * @param tagSize Size of the AprilTag in meters.
     * @param fx      X focal length.
     * @param fy      Y focal length.
     * @param cx      X center pixel coordinate.
     * @param cy      Y center pixel coordinate.
     */
    public AprilTagPipeline(double tagSize, double fx, double fy, double cx, double cy) {
        this.tagSize  = tagSize;
        this.tagSizeX = tagSize;
        this.tagSizeY = tagSize;
        this.fx       = fx;
        this.fy       = fy;
        this.cx       = cx;
        this.cy       = cy;

        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalize method
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
    }

    @Override
	protected void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if (nativeApriltagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert to greyscale
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

        synchronized (decimationSync) {
            if (needToSetDecimation) {
                AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                needToSetDecimation = false;
            }
        }

        // Run AprilTag
        detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagSize, fx, fy, cx, cy);

        synchronized (detectionsUpdateSync) {
            detectionsUpdate = detections;
        }

        // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
        // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
        for (AprilTagDetection detection : detections) {
            Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagSizeX, tagSizeY);
            drawAxisMarker(input, tagSizeY / 2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
            draw3dCubeMarker(input, tagSizeX, tagSizeX, tagSizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
        }

        return input;
    }

    /**
     * Sets the decimation value.
     *
     * @param decimation Decimation value to set.
     */
    public void setDecimation(float decimation) {
        synchronized (decimationSync) {
            this.decimation     = decimation;
            needToSetDecimation = true;
        }
    }

    /**
     * Retrieves the latest detections.
     *
     * @return ArrayList of AprilTagDetections.
     */
    public ArrayList<AprilTagDetection> getLatestDetections() {
        return detections;
    }

    /**
     * Retrieves the updated detections.
     *
     * @return ArrayList of AprilTagDetections.
     */
    public ArrayList<AprilTagDetection> getDetectionsUpdate() {
        synchronized (detectionsUpdateSync) {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    /**
     * Constructs the camera matrix.
     */
    private void constructMatrix() {
        // Construct the camera matrix.
        //
        // --         --
        // | fx   0   cx |
        // | 0    fy  cy |
        // | 0    0   1  |
        // --         --

        cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);

        cameraMatrix.put(0, 0, fx);
        cameraMatrix.put(0, 1, 0);
        cameraMatrix.put(0, 2, cx);

        cameraMatrix.put(1, 0, 0);
        cameraMatrix.put(1, 1, fy);
        cameraMatrix.put(1, 2, cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2, 1, 0);
        cameraMatrix.put(2, 2, 1);
    }

    /**
     * Draws a 3D axis marker on a detection.
     *
     * @param buf          the RGB buffer on which to draw the marker
     * @param length       the length of each of the marker 'poles'
     * @param thickness    the thickness of the lines
     * @param rvec         the rotation vector of the detection
     * @param tvec         the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    private void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(new Point3(0, 0, 0), new Point3(length, 0, 0), new Point3(0, length, 0), new Point3(0, 0, -length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    /**
     * Draws a 3D cube marker on the input image using the provided rotation vector, translation vector,
     * and camera matrix. The cube is defined by its length, width, and height. The cube's bottom face
     * is drawn by connecting the bottom points, and the top face is drawn by connecting the corresponding
     * top points. Pillars connecting the top and bottom faces are also drawn. All lines are drawn with
     * the specified thickness and color.
     *
     * @param buf          The input image on which to draw the 3D cube marker.
     * @param length       The length of the cube along the z-axis.
     * @param tagWidth     The width of the cube along the x-axis.
     * @param tagHeight    The height of the cube along the y-axis.
     * @param thickness    The thickness of the lines used to draw the cube.
     * @param rvec         The rotation vector representing the cube's orientation in 3D space.
     * @param tvec         The translation vector representing the cube's position in 3D space.
     * @param cameraMatrix The camera intrinsic matrix used for projection.
     */
    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix) {
        MatOfPoint3f axis = new MatOfPoint3f(new Point3(-tagWidth / 2, tagHeight / 2, 0), new Point3(tagWidth / 2, tagHeight / 2, 0), new Point3(tagWidth / 2, -tagHeight / 2, 0), new Point3(-tagWidth / 2, -tagHeight / 2, 0), new Point3(-tagWidth / 2, tagHeight / 2, -length), new Point3(tagWidth / 2, tagHeight / 2, -length), new Point3(tagWidth / 2, -tagHeight / 2, -length), new Point3(-tagWidth / 2, -tagHeight / 2, -length));

        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for (int i = 0; i < 4; i++) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i + 4], blue, thickness);
        }

        // Base lines
        for (int i = 0; i < 4; i++) {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[(i + 1) % 4], blue, thickness);
        }

        // Top lines
        for (int i = 0; i < 4; i++) {
            Imgproc.line(buf, projectedPoints[i + 4], projectedPoints[(i + 1) % 4 + 4], green, thickness);
        }
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points       the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX     the original width of the tag
     * @param tagsizeY     the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX, double tagsizeY) {
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX / 2, tagsizeY / 2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX / 2, -tagsizeY / 2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX / 2, -tagsizeY / 2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose {
        Mat rvec;
        Mat tvec;

        public Pose() {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec) {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}