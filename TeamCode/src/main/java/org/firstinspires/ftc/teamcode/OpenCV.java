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
import org.opencv.core.Point;
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

@SuppressWarnings("unused")
@Autonomous(name = "Auto Block Collector")
public class OpenCV extends LinearOpMode {
    private MecanumDrive drive;
    private OpenCvCamera camera;
    private BlockDetectionPipeline pipeline;
    private AprilTagProcessor aprilTag;
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;
    private static final double PICKUP_DISTANCE = 8.0; // inches
    private final ElapsedTime timer = new ElapsedTime();
    private RobotState currentState;
    private double targetX, targetY;

    public enum RobotState {
        WANDER,
        APPROACH_BLOCK,
        PICKUP,
        RETURN_TO_START,
        SCORE
    }

    @Override
    public void runOpMode() {
        // Initialize
        initVision();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        currentState = RobotState.WANDER;

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            BlockDetectionResult detection = pipeline.getLatestResult();

            switch (currentState) {
                case WANDER:
                    executeWanderPattern();
                    if (detection != null && detection.isBlockDetected) {
                        currentState = RobotState.APPROACH_BLOCK;
                        timer.reset();
                    }
                    break;

                case APPROACH_BLOCK:
                    if (detection != null && detection.isBlockDetected) {
                        navigateToBlock(detection);
                        if (isAtPickupPosition()) {
                            currentState = RobotState.PICKUP;
                            timer.reset();
                        }
                    } else {
                        currentState = RobotState.WANDER;
                    }
                    break;

                case PICKUP:
                    executePickup();
                    if (isPickupComplete()) {
                        currentState = RobotState.RETURN_TO_START;
                        timer.reset();
                    }
                    break;

                case RETURN_TO_START:
                    AprilTagDetection tag = getVisibleAprilTag();
                    if (tag != null) {
                        returnToStart();
                        if (isAtStartPosition()) {
                            currentState = RobotState.SCORE;
                            timer.reset();
                        }
                    }
                    break;

                case SCORE:
                    executeScoring();
                    if (isScoringComplete()) {
                        currentState = RobotState.WANDER;
                        timer.reset();
                    }
                    break;
            }

            telemetry.addData("State", currentState);
            telemetry.addData("Block Detected", detection != null && detection.isBlockDetected);
            telemetry.addData("Block Type", detection != null ? detection.blockType : "None");
            telemetry.addData("Timer", "%.1f", timer.seconds());
            telemetry.update();

            sleep(10);
        }
    }

    private void initVision() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new BlockDetectionPipeline();
        camera.setPipeline(pipeline);

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
            if (!opModeIsActive() || timer.seconds() >= 30) break;

            drive.actionBuilder(drive.pose)
                    .splineTo(point, Math.atan2(point.y, point.x))
                    .waitSeconds(1)
                    .build();
        }
    }

    private void executePickup() {
        drive.controlArm(-0.5, 0);
        sleep(1000);
        drive.intakeMove(1.0);
        sleep(1500);
        drive.controlArm(0.5, 0);
        sleep(1000);
        drive.intakeMove(0);
    }

    private void executeScoring() {
        drive.linearMove(1.0);
        sleep(2000);
        drive.tiltTray(1.0, 500);
        drive.linearMove(-1.0);
        sleep(2000);
        drive.linearMove(0);
    }

    private void navigateToBlock(BlockDetectionResult detection) {
        targetX = detection.x;
        targetY = detection.y;

        drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(targetX, targetY), detection.heading)
                .build();
    }

    private void returnToStart() {
        drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(0, 0), 0)
                .build();
    }

    private boolean isAtPickupPosition() {
        return Math.abs(drive.pose.position.x - targetX) < 2.0 &&
                Math.abs(drive.pose.position.y - targetY) < 2.0;
    }

    private boolean isPickupComplete() {
        return timer.seconds() > 3.0;
    }

    private boolean isAtStartPosition() {
        return Math.abs(drive.pose.position.x) < 2.0 &&
                Math.abs(drive.pose.position.y) < 2.0;
    }

    private boolean isScoringComplete() {
        return timer.seconds() > 4.0;
    }

    private AprilTagDetection getVisibleAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        return !detections.isEmpty() ? detections.get(0) : null;
    }

    class BlockDetectionPipeline extends OpenCvPipeline {
        private BlockDetectionResult latestResult = null;
        private static final double MIN_CONTOUR_AREA = 1000;

        @Override
        public Mat processFrame(Mat input) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(input, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar[] lowerBounds = {
                    new Scalar(20, 100, 100),  // Yellow
                    new Scalar(100, 100, 100)  // Blue
            };
            Scalar[] upperBounds = {
                    new Scalar(30, 255, 255),  // Yellow
                    new Scalar(130, 255, 255)  // Blue
            };

            for (int i = 0; i < 2; i++) {
                Mat colorMask = new Mat();
                Core.inRange(hsvFrame, lowerBounds[i], upperBounds[i], colorMask);

                List<MatOfPoint> contours = new ArrayList<>();
                Mat hierarchy = new Mat();
                Imgproc.findContours(colorMask, contours, hierarchy,
                        Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

                MatOfPoint largestContour = null;
                double maxArea = MIN_CONTOUR_AREA;

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                if (largestContour != null) {
                    Moments moments = Imgproc.moments(largestContour);
                    double cX = moments.get_m10() / moments.get_m00();
                    double cY = moments.get_m01() / moments.get_m00();

                    BlockType type = (i == 0) ? BlockType.YELLOW : BlockType.BLUE;

                    latestResult = new BlockDetectionResult(
                            true,
                            type,
                            cX,
                            cY,
                            Math.atan2(cY - CAMERA_HEIGHT/2.0, cX - CAMERA_WIDTH/2.0)
                    );

                    Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0,255,0), -1);
                    Imgproc.drawContours(input, Arrays.asList(largestContour), -1, new Scalar(255,0,0), 2);
                }

                colorMask.release();
                hierarchy.release();
            }

            hsvFrame.release();
            return input;
        }

        public BlockDetectionResult getLatestResult() {
            return latestResult;
        }
    }

    enum BlockType {
        YELLOW, BLUE
    }

    static class BlockDetectionResult {
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
}