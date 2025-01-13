package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutoModeCode", group = "Autonomous")
public class BasketAuto extends LinearOpMode {
    // Declare four motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Wait for the game to start
        telemetry.addData("Status", "Waiting for start command...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Autonomous logic

            // Strafe right 12.5 cm
            telemetry.addData("Step", "Strafing right 12.5 cm");
            telemetry.update();
            driveStrafe(0.5, 12.5);

            // Move forward 90 cm
            telemetry.addData("Step", "Driving forward 90 cm");
            telemetry.update();
            driveForward(0.5, 90);

            // Turn left 45 degrees in place
            telemetry.addData("Step", "Turning left 45 degrees");
            telemetry.update();
            turnLeft(0.3, 45);

            // Move forward 21 cm
            telemetry.addData("Step", "Driving forward 21 cm");
            telemetry.update();
            driveForward(0.5, 21);

            // Draw first emoji
            telemetry.addData("Step", "Drawing first emoji: :) ");
            telemetry.update();
            sleep(2000);

            // Move backward 21 cm
            telemetry.addData("Step", "Driving backward 21 cm");
            telemetry.update();
            driveBackward(0.5, 21);

            // Turn right 135 degrees in place
            telemetry.addData("Step", "Turning right 135 degrees");
            telemetry.update();
            turnRight(0.3, 135);

            // Draw second emoji
            telemetry.addData("Step", "Drawing second emoji: :D ");
            telemetry.update();
            sleep(2000);

            // Turn left 135 degrees in place
            telemetry.addData("Step", "Turning left 135 degrees");
            telemetry.update();
            turnLeft(0.3, 135);

            // Move forward 21 cm
            telemetry.addData("Step", "Driving forward 21 cm");
            telemetry.update();
            driveForward(0.5, 21);

            // Draw first emoji again
            telemetry.addData("Step", "Drawing first emoji again: :) ");
            telemetry.update();
            sleep(2000);

            // Complete
            telemetry.addData("Status", "Autonomous mode complete");
            telemetry.update();
        }
    }

    // Helper function: Control motor operation
    private void drive(double lfPower, double rfPower, double lbPower, double rbPower, long duration) throws InterruptedException {
        leftFront.setPower(lfPower);
        rightFront.setPower(rfPower);
        leftBack.setPower(lbPower);
        rightBack.setPower(rbPower);
        sleep(duration);
        stopDriving();
    }

    // Move forward
    private void driveForward(double speed, long duration) throws InterruptedException {
        drive(speed, speed, speed, speed, duration);
    }

    // Move backward
    private void driveBackward(double speed, long duration) throws InterruptedException {
        drive(-speed, -speed, -speed, -speed, duration);
    }

    // Turn left
    private void turnLeft(double speed, double angle) throws InterruptedException {
        double time = angle / 90 * 1000; // Assume 90 degrees/s speed, calculate time
        drive(-speed, speed, -speed, speed, (long) time);
    }

    // Turn right
    private void turnRight(double speed, double angle) throws InterruptedException {
        double time = angle / 90 * 1000; // Assume 90 degrees/s speed, calculate time
        drive(speed, -speed, speed, -speed, (long) time);
    }

    // Strafe sideways
    private void driveStrafe(double speed, double distance) throws InterruptedException {
        double time = distance / 50 * 1000; // Assume 50 cm/s speed, calculate time
        drive(speed, -speed, -speed, speed, (long) time);
    }

    // Stop all motors
    private void stopDriving() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}