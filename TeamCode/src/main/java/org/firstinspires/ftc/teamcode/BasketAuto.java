package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoModeCode", group = "Autonomous")
public class BasketAuto extends LinearOpMode {
    // Declare four motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Declare linear slide motors
    private DcMotor linearLeft;
    private DcMotor linearRight;

    // Declare servo motors
    private Servo clawServo;
    private Servo clawArmServo;

    // Correction factors
    private static final double distanceCorrectionFactor = 1 / 0.9; // Distance correction factor
    private static final double angleCorrectionFactor = 90.0 / 45.0; // Angle correction factor

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        linearLeft.setDirection(DcMotor.Direction.FORWARD);
        linearRight.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start
        telemetry.addData("Status", "Waiting for start command...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // Autonomous logic

            // Strafe right 10 cm
            telemetry.addData("Step", "Strafing right 10 cm");
            telemetry.update();
            strafeRight(0.5, 10);

            // Wait for 5 seconds
            telemetry.addData("Step", "Waiting for 5 seconds");
            telemetry.update();
            sleep(5000);

            // Raise linear slide by 20 cm
            telemetry.addData("Step", "Raising linear slide 20 cm");
            telemetry.update();
            moveLinear(-0.5, 2000); // Assume 1 second raises 1 cm

            // Wait for 5 seconds
            telemetry.addData("Step", "Waiting for 5 seconds");
            telemetry.update();
            sleep(5000);

            // Lower linear slide by 20 cm
            telemetry.addData("Step", "Lowering linear slide 20 cm");
            telemetry.update();
            moveLinear(0.5, 2000);

            // Turn right 90 degrees
            telemetry.addData("Step", "Turning right 90 degrees");
            telemetry.update();
            turnRight(0.3, 90);

            // Wait for 5 seconds
            telemetry.addData("Step", "Waiting for 5 seconds");
            telemetry.update();
            sleep(5000);

            // Drive forward 10 cm
            telemetry.addData("Step", "Driving forward 10 cm");
            telemetry.update();
            driveForward(0.5, 10);

            // Drive backward 10 cm
            telemetry.addData("Step", "Driving backward 10 cm");
            telemetry.update();
            driveBackward(0.5, 10);

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

    // Strafe right
    private void strafeRight(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * distanceCorrectionFactor;
        double time = correctedDistance / 50 * 1000; // Assume speed of 50 cm/s, calculate time
        drive(speed, -speed, -speed, speed, (long) time);
    }

    // Strafe left
    private void strafeLeft(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * distanceCorrectionFactor;
        double time = correctedDistance / 50 * 1000; // Assume speed of 50 cm/s, calculate time
        drive(-speed, speed, speed, -speed, (long) time);
    }

    // Drive forward
    private void driveForward(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * distanceCorrectionFactor;
        double time = correctedDistance / 50 * 1000; // Assume speed of 50 cm/s, calculate time
        drive(speed, speed, speed, speed, (long) time);
    }

    // Drive backward
    private void driveBackward(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * distanceCorrectionFactor;
        double time = correctedDistance / 50 * 1000; // Assume speed of 50 cm/s, calculate time
        drive(-speed, -speed, -speed, -speed, (long) time);
    }

    // Turn left
    private void turnLeft(double speed, double angle) throws InterruptedException {
        double correctedAngle = angle * angleCorrectionFactor;
        double time = correctedAngle / 90 * 1000; // Assume speed of 90 degrees/s, calculate time
        drive(-speed, speed, -speed, speed, (long) time);
    }

    // Turn right
    private void turnRight(double speed, double angle) throws InterruptedException {
        double correctedAngle = angle * angleCorrectionFactor;
        double time = correctedAngle / 90 * 1000; // Assume speed of 90 degrees/s, calculate time
        drive(speed, -speed, speed, -speed, (long) time);
    }

    // Control linear slide
    private void moveLinear(double speed, long duration) throws InterruptedException {
        linearLeft.setPower(speed);
        linearRight.setPower(speed);
        sleep(duration);
        stopLinear();
    }

    // Stop linear slide
    private void stopLinear() {
        linearLeft.setPower(0);
        linearRight.setPower(0);
    }

    // Control claw servo motor
    private void moveClaw(double position) {
        clawServo.setPosition(position);
    }

    // Control claw arm servo motor
    private void moveClawArm(double position) {
        clawArmServo.setPosition(position);
    }

    // Stop all motors
    private void stopDriving() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}