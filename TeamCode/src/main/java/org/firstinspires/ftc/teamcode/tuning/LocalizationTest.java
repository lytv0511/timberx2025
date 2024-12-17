package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public class LocalizationTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();
            while (opModeIsActive()) {
                boolean leftBumperHeld = gamepad1.left_bumper;
                boolean rightBumperHeld = gamepad1.right_bumper;

                if (!leftBumperHeld && !rightBumperHeld) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    gamepad1.left_stick_y,
                                    gamepad1.left_stick_x
                            ),
                            gamepad1.right_stick_x
                    ));
                } else {
                    // Stop movement when bumper is held (or maintain state as needed)
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                }

                // Update pose estimation
                drive.updatePoseEstimate();

                // Telemetry updates
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                // Handle left bumper functionality
                if (leftBumperHeld) {
                    while (gamepad1.left_stick_y > 0) {
                        drive.linearMove(-gamepad1.left_stick_y);
                    }
                    while (gamepad1.left_stick_y < 0) {
                        drive.linearMove(-gamepad1.left_stick_y);
                    }
                    drive.linearMove(0);
                }

                // Handle right bumper functionality
                if (rightBumperHeld) {
                        if (gamepad1.left_stick_y > 0) {
                            drive.controlArm(gamepad1.left_stick_y/2, 0); // Move the arm up
                        } else if (gamepad1.left_stick_y < 0) {
                            drive.controlArm(gamepad1.left_stick_y/2, 0); // Move the arm down
                        } else if (gamepad1.left_stick_y == 0) {
                            drive.controlArm(0, 1);
                        }
                }


                // Trigger controls
                if (gamepad1.right_trigger > 0.0) {
                    drive.intakeMove(gamepad1.right_trigger);
                    while (gamepad1.right_trigger > 0.0) {
                        drive.intakeMove(100);
                    }
                    drive.intakeMove(0);
                }
                if (gamepad1.left_trigger > 0.9) {
                    while (gamepad1.left_trigger > 0.9) {
                        drive.tiltTray(0.1, 500);
                    }
                } else if (gamepad1.left_trigger == 0) {
                    drive.tiltTray(0, 500);
                }
            }
        } else {
            throw new RuntimeException();
        }
    }
}
