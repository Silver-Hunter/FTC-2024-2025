package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
/*
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left

 */

@TeleOp(name="Driver Control", group="Linear OpMode")

public class DriveTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor guideRailMotor = null;
    private DcMotor linearSlide = null;
    double ticks = 537.6;
    double newTarget;
    private Servo Claw = null;
    private Servo Wrist = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        guideRailMotor = hardwareMap.get(DcMotor.class, "guideRailMotor");
        linearSlide = hardwareMap.get(DcMotor.class, "liftMotor");
        Claw = hardwareMap.get(Servo.class, "Claw");
        Wrist = hardwareMap.get(Servo.class, "Wrist");


        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        guideRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            double max;

            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            if (gamepad1.left_trigger > 0.000) {
                axial = axial * 0.55;
                lateral = lateral * 0.55;
                yaw = yaw * 0.55;
            }
            if (gamepad1.left_trigger > 0.000 && gamepad1.left_trigger < 0.001) {
                axial = axial / 0.55;
                lateral = lateral / 0.55;
                yaw = yaw / 0.55;
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = (axial + 0.15) - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            frontLeftDrive.setPower(Math.pow(leftFrontPower, 3));
            frontRightDrive.setPower(Math.pow(rightFrontPower, 3));
            backLeftDrive.setPower(Math.pow(leftBackPower, 3));
            backRightDrive.setPower(Math.pow(rightBackPower, 3));

            if (gamepad2.x){
                Claw.setPosition(0);
            }

            if (gamepad2.a){
                Wrist.setPosition(1);
            }

            if (gamepad2.y){
                Wrist.setPosition(0);
            }

            if (gamepad2.b){
                Claw.setPosition(1);
            }

            if (gamepad2.dpad_up){
                encoder(4);
            }

            if (gamepad2.dpad_down){
                encoder(-4);
            }

            linearSlide.setPower(gamepad2.left_stick_y);

            if (linearSlide.getCurrentPosition() > 1000 || linearSlide.getCurrentPosition() > -1000){
                linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Linear Slide Ticks: ", + ticks);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }

        if (linearSlide.getCurrentPosition() > 3300){
            linearSlide.setPower(0);
        }
    }

    private void encoder(int turnage) {
        guideRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks/turnage;
        guideRailMotor.setTargetPosition((int)newTarget);
        guideRailMotor.setPower(0.5);
        guideRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
