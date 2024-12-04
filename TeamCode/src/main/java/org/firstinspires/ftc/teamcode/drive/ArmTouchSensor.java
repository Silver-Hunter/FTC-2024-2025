package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

/*
 * This OpMode demonstrates how to use a REV Robotics Touch Sensor, REV Robotics Magnetic Limit Switch, or other device
 * that implements the TouchSensor interface. Any touch sensor that connects its output to ground when pressed
 * (known as "active low") can be configured as a "REV Touch Sensor". This includes REV's Magnetic Limit Switch.
 *
 * The OpMode assumes that the touch sensor is configured with a name of "sensor_touch".
 *
 * A REV Robotics Touch Sensor must be configured on digital port number 1, 3, 5, or 7.
 * A Magnetic Limit Switch can be configured on any digital port.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Arm Reach Limit", group = "Sensor")

public class ArmTouchSensor extends LinearOpMode {
    TouchSensor touchSensor;  // Touch sensor Object
    private DcMotor linearSlide = null;
    private DcMotor guideRailMotor = null;
    double ticks;
    double newTarget;




    @Override
    public void runOpMode() {

        // get a reference to our touchSensor object.
        touchSensor = hardwareMap.get(TouchSensor.class, "TouchSensor");
        linearSlide = hardwareMap.get(DcMotor.class, "liftMotor");
        guideRailMotor = hardwareMap.get(DcMotor.class, "guideRailMotor");

        guideRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlide.setPower(gamepad2.left_stick_y);


        if (linearSlide.getCurrentPosition() > 3300){
            linearSlide.setPower(0);
        }



        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            linearSlide.setPower(gamepad2.left_stick_y);

            // send the info back to driver station using telemetry function.
//            if (guideRailMotor.getCurrentPosition() > 1000){
//                linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }

                if (linearSlide.getCurrentPosition() > -1700) {
                    linearSlide.setPower(0);
                    linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                }

            telemetry.addData("Touch Sensor", "Is Pressed");
            telemetry.addData("Linear Slide Ticks: ", linearSlide.getCurrentPosition());
            telemetry.update();
        }

    }
}
