package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutoModeBarCode", group = "Autonomous")
public class BarAuto extends LinearOpMode {

    // Declare motors for drivetrain
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    // Declare motors for linear slide
    private DcMotor linearLeft;
    private DcMotor linearRight;

    // Declare servos for claw control
    private Servo clawServo;
    private Servo clawArmServo;

    // Correction factors
    private static final double DISTANCE_CORRECTION_FACTOR = 1 / 0.9; // Distance correction
    private static final double ANGLE_CORRECTION_FACTOR = 90.0 / 45.0; // Angle correction

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        initializeHardware();

        // Wait for start command
        telemetry.addData("Status", "Waiting for start command...");
        telemetry.update();
        waitForStart();

        // Autonomous logic
        if (opModeIsActive()) {
            telemetry.addData("Status", "Running autonomous mode...");
            telemetry.update();

            // Add your autonomous steps here
            // Example: driveForward(0.5, 50); // Drive forward at 50% speed for 50 cm
        }
    }

    // Initialize hardware components
    private void initializeHardware() {
        // Map motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        linearLeft = hardwareMap.get(DcMotor.class, "linearLeft");
        linearRight = hardwareMap.get(DcMotor.class, "linearRight");

        // Map servos
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawArmServo = hardwareMap.get(Servo.class, "clawArmServo");

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        linearLeft.setDirection(DcMotor.Direction.FORWARD);
        linearRight.setDirection(DcMotor.Direction.REVERSE);
    }

    // Drive the robot with specified power for a duration
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
        double correctedDistance = distance * DISTANCE_CORRECTION_FACTOR;
        long time = (long) ((correctedDistance / 50) * 1000); // Assume 50 cm/s speed
        drive(speed, -speed, -speed, speed, time);
    }

    // Strafe left
    private void strafeLeft(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * DISTANCE_CORRECTION_FACTOR;
        long time = (long) ((correctedDistance / 50) * 1000);
        drive(-speed, speed, speed, -speed, time);
    }

    // Drive forward
    private void driveForward(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * DISTANCE_CORRECTION_FACTOR;
        long time = (long) ((correctedDistance / 50) * 1000);
        drive(speed, speed, speed, speed, time);
    }

    // Drive backward
    private void driveBackward(double speed, double distance) throws InterruptedException {
        double correctedDistance = distance * DISTANCE_CORRECTION_FACTOR;
        long time = (long) ((correctedDistance / 50) * 1000);
        drive(-speed, -speed, -speed, -speed, time);
    }

    // Turn left
    private void turnLeft(double speed, double angle) throws InterruptedException {
        double correctedAngle = angle * ANGLE_CORRECTION_FACTOR;
        long time = (long) ((correctedAngle / 90) * 1000); // Assume 90°/s speed
        drive(-speed, speed, -speed, speed, time);
    }

    // Turn right
    private void turnRight(double speed, double angle) throws InterruptedException {
        double correctedAngle = angle * ANGLE_CORRECTION_FACTOR;
        long time = (long) ((correctedAngle / 90) * 1000);
        drive(speed, -speed, speed, -speed, time);
    }

    // Move linear slide
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

    // Move claw servo
    private void moveClaw(double position) {
        clawServo.setPosition(position);
    }

    // Move claw arm servo
    private void moveClawArm(double position) {
        clawArmServo.setPosition(position);
    }

    // Stop all drivetrain motors
    private void stopDriving() {
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}