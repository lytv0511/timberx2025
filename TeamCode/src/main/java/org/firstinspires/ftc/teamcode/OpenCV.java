package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Auto Block Collector")
public class OpenCV extends LinearOpMode {
    private MecanumDrive drive;
    private OpenCvCamera camera;
    private AprilTagProcessor aprilTag;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    double cX = 0, cY = 0, width = 0;
    private static final double PICKUP_DISTANCE = 8.0; // inches
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Initialize RoadRunner drive
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Initialize camera and vision
        initVision();

        waitForStart();

        while (opModeIsActive()) {
            // Update robot localization
            drive.updatePoseEstimate();

            // Scan for blocks
            BlockDetectionResult detection = processFrame();

            if (detection.isBlockDetected) {
                if (detection.blockType == BlockType.YELLOW || detection.blockType == BlockType.BLUE) {
                    // Navigate to block
                    navigateToBlock(detection);

                    // Pickup sequence
                    executePickup();

                    // Return to starting position using AprilTag
                    returnToStart();

                    // Score sequence
                    executeScoring();
                }
            } else {
                // Wander pattern using RoadRunner
                executeWanderPattern();
            }
        }
    }

    private void initVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new BlockDetectionPipeline());

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT));
        builder.addProcessor(aprilTag);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });
    }

    private void executeWanderPattern() {
        List<Vector2d> searchPoints = Arrays.asList(
                new Vector2d(24, 24),
                new Vector2d(24, -24),
                new Vector2d(-24, -24),
                new Vector2d(-24, 24)
        );

        for (Vector2d point : searchPoints) {
            drive.actionBuilder(drive.pose)
                    .splineTo(point, Math.atan2(point.y, point.x))
                    .waitSeconds(1)
                    .build();
        }
    }

    private void executePickup() {
        // Arm down
        drive.controlArm(-0.5);
        sleep(1000);

        // Activate intake
        drive.intakeMove(1.0);
        sleep(1500);

        // Arm up
        drive.controlArm(0.5);
        sleep(1000);

        // Stop intake
        drive.intakeMove(0);
    }

    private void executeScoring() {
        // Extend linear slides
        drive.linearMove(1.0);
        sleep(2000);

        // Tilt tray to score
        drive.tiltTray(1.0, 500);

        // Retract slides
        drive.linearMove(-1.0);
        sleep(2000);
        drive.linearMove(0);
    }

    private void navigateToBlock(BlockDetectionResult detection) {
        // Use RoadRunner to navigate to block position
        drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(detection.x, detection.y), detection.heading)
                .build();
    }

    private void returnToStart() {
        // Use AprilTag to locate and navigate back to starting position
        AprilTagDetection tag = getVisibleAprilTag();
        if (tag != null) {
            drive.actionBuilder(drive.pose)
                    .splineTo(new Vector2d(0, 0), 0)
                    .build();
        }
    }

    class BlockDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Color detection for Red, Yellow, Blue
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // HSV ranges for each color
            Scalar[] lowerBounds = {
                    new Scalar(0, 100, 100),   // Red
                    new Scalar(20, 100, 100),  // Yellow
                    new Scalar(100, 100, 100)  // Blue
            };
            Scalar[] upperBounds = {
                    new Scalar(10, 255, 255),  // Red
                    new Scalar(30, 255, 255),  // Yellow
                    new Scalar(130, 255, 255)  // Blue
            };

            // Process each color mask
            for (int i = 0; i < 3; i++) {
                Mat colorMask = new Mat();
                Core.inRange(hsvFrame, lowerBounds[i], upperBounds[i], colorMask);

                // Find contours and process largest
                List<MatOfPoint> contours = new ArrayList<>();
                Imgproc.findContours(colorMask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                if (!contours.isEmpty()) {
                    MatOfPoint largest = findLargestContour(contours);
                    // Process contour and update detection data
                    processContour(largest, i);
                }
            }

            return input;
        }
    }

    // Add enum for block types
    enum BlockType {
        RED, YELLOW, BLUE
    }

    // Add detection result class
    class BlockDetectionResult {
        public boolean isBlockDetected;
        public BlockType blockType;
        public double x;
        public double y;
        public double heading;

        public BlockDetectionResult(boolean detected, BlockType type, double x, double y, double heading) {
            this.isBlockDetected = detected;
            this.blockType = type;
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    // Add helper methods to BlockDetectionPipeline
    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        return largestContour;
    }

    private void processContour(MatOfPoint contour, int colorIndex) {
        if (contour != null) {
            Moments moments = Imgproc.moments(contour);
            cX = moments.get_m10() / moments.get_m00();
            cY = moments.get_m01() / moments.get_m00();
            width = Imgproc.boundingRect(contour).width;
        }
    }

    // Add method to get visible AprilTag
    private AprilTagDetection getVisibleAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        return !detections.isEmpty() ? detections.get(0) : null;
    }

    // Add method to process camera frame
    private BlockDetectionResult processFrame() {
        // Process the latest frame from camera
        BlockDetectionResult result = new BlockDetectionResult(false, null, 0, 0, 0);
        // Add processing logic here
        return result;
    }
}