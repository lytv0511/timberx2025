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
            drive.leftFront.setPower(0);
            drive.rightFront.setPower(0);
            while (opModeIsActive()) while (opModeIsActive()) {
                // Check if bumpers are held
                boolean leftBumperHeld = gamepad1.left_bumper;
                boolean rightBumperHeld = gamepad1.right_bumper;

                // Disable standard movement when either bumper is held
                if (!leftBumperHeld && !rightBumperHeld) {
                    // Normal driving controls (only active if no bumpers are held)
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.right_stick_x
                            ),
                            -gamepad1.left_stick_x
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

                // Handle left bumper functionality
                if (leftBumperHeld) {
                    while (gamepad1.left_stick_y > 0.2) {
                        drive.linearMove(gamepad1.right_stick_y);
                    }
                    while (gamepad1.left_stick_y < -0.2) {
                        drive.linearMove(-gamepad1.right_stick_y);
                    }
                    drive.linearMove(0);  // Ensure stop after bumper release
                }

                // Handle right bumper functionality
                if (rightBumperHeld) {
                    double stickInput = gamepad1.left_stick_y;

                    if (stickInput > 0.2) {
                        drive.controlArm(stickInput); // Move the arm up
                    } else if (stickInput < -0.2) {
                        drive.controlArm(stickInput); // Move the arm down
                    } else {
                        drive.controlArm(0); // Stop the arm
                    }
                }

                // Trigger controls
                if (gamepad1.right_trigger > 0.0) {
                    drive.intakeMove(gamepad1.right_trigger);
                    while (gamepad1.right_trigger > 0.0) {
                        // Wait while the trigger is held
                    }/**/
                    drive.intakeMove(0);
                }
                if (gamepad1.left_trigger > 0.9) {
                    drive.tiltTray(1.0, 500); // Adjust speed and duration as needed
                }

                // Telemetry packet for dashboard visualization
                TelemetryPacket packet = new TelemetryPacket();
                packet.fieldOverlay().setStroke("#3F51B5");
                Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
