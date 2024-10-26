package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Sensor: HuskyLens", group = "Sensor")
public class HuskyLensTest extends LinearOpMode {

    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;
    public ElapsedTime mClock = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    static final double     COUNTS_PER_MOTOR_REV    = 537.6 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.9 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.4;


    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        telemetry.addData("Starting at",  "%7d :%7d",
                frontLeftDrive.getCurrentPosition(),
                frontRightDrive.getCurrentPosition(),
                backLeftDrive.getCurrentPosition(),
                backRightDrive.getCurrentPosition());

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.update();
        waitForStart();

        /*
         * Looking for AprilTags per the call to selectAlgorithm() above.  A handy grid
         * for testing may be found at https://wiki.dfrobot.com/HUSKYLENS_V1.0_SKU_SEN0305_SEN0336#target_20.
         *
         * Note again that the device only recognizes the 36h11 family of tags out of the box.
         */
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            /*
             * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
             * Block represents the outline of a recognized object along with its ID number.
             * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
             * referenced in the header comment above for more information on IDs and how to
             * assign them to objects.
             *
             * Returns an empty array if no objects are seen.
             */
                /*HuskyLens.Block[] blocks = huskyLens.blocks();
                telemetry.addData("Block count", blocks.length);
                for (int i = 0; i < blocks.length; i++) {
                    telemetry.addData("Block", blocks[i].toString());
                }*/

            int x_pos = -1; //Todo change tp -1
            int y_pos = 1;
            int id;
            int width;
            int height;
            int top;
            int left;
            //String color;



            while (opModeIsActive()) {
                mClock.reset();

                //get label start
                //while (x_pos == -1 && mClock.seconds() < 0) {//mClock.seconds
                telemetry.addData("mClock", mClock.seconds());
                telemetry.update();
                HuskyLens.Block[] blocks = huskyLens.blocks();

                if (blocks.length > 0) {
                    telemetry.addData("Block count", blocks.length);
                    int[] distance = new int[blocks.length];

                    for (int i = 0; i < blocks.length; i++ ) {
                        if ( blocks[i].id == 1 || blocks[i].id == 2 || blocks[i].id == 3) {
                            //Todo id #
                            x_pos = blocks[i].x;
                            y_pos = blocks[i].y;
                            id = blocks[i].id;
                            width = blocks[i].width;
                            height = blocks[i].height;
                            top = blocks[i].top;
                            left = blocks[i].left;
                            telemetry.addData("id:", id);
                            telemetry.addData("   x_pos: ", x_pos);
                            telemetry.addData("   y_pos: ", y_pos);
                            telemetry.addData("   width: ", width);
                            telemetry.addData("   height: ", height);
                            telemetry.addData("   top: ", top);
                            telemetry.addData("   left: ", left);

                            distance[i] = x_pos - 155;
                        } //end assigning x_pos
                    } //end getting block id

                    encoderDrive(DRIVE_SPEED,  18,  18, 2);

                    int smallestDistance = 1000;
                    for (int i = 0; i < distance.length; i++) {
                        if (Math.abs(distance[i]) < Math.abs(smallestDistance) ) {
                            smallestDistance = distance[i];
                        }
                    }
                    if (smallestDistance > 0) {
                        encoderDrive(DRIVE_SPEED,  -0.2,  0.2, 1.1);
                        sleep(5000);

                    }
                    else {
                        encoderDrive(DRIVE_SPEED,  0.2,  -0.2, 1.1);
                        sleep(5000);

                    }
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                } //end getting blocks
                encoderDrive(DRIVE_SPEED,  45,  45, 3);
                encoderDrive(DRIVE_SPEED,  -10,  10, 1);
                encoderDrive(DRIVE_SPEED,  16,  16, 1);
                encoderDrive(DRIVE_SPEED,  -10,  10, 1);
                encoderDrive(DRIVE_SPEED,  55,  55, 3);




                break; //break whileOpMode to start autonomous
            }
            telemetry.update();
        }
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int frontLeftTarget;
        int frontRightTarget;
        int backLeftTarget;
        int backRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            frontLeftTarget = frontLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            frontRightTarget = frontRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            backLeftTarget = backLeftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            backRightTarget = backRightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            frontLeftDrive.setTargetPosition(frontLeftTarget);
            frontRightDrive.setTargetPosition(frontRightTarget);
            backLeftDrive.setTargetPosition(backLeftTarget);
            backRightDrive.setTargetPosition(backRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            ElapsedTime runtime = new ElapsedTime();
            runtime.reset();
            frontLeftDrive.setPower(Math.abs(speed));
            frontRightDrive.setPower(Math.abs(speed));
            backLeftDrive.setPower(Math.abs(speed));
            backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy() && backRightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", frontLeftTarget,  frontRightTarget, backLeftTarget, backRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeftDrive.getCurrentPosition(), frontRightDrive.getCurrentPosition(), backLeftDrive.getCurrentPosition(), backRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeftDrive.setPower(0);
            frontRightDrive.setPower(0);
            backLeftDrive.setPower(0);
            backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
}


