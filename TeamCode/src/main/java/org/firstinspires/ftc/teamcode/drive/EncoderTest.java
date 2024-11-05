package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Encoder TEST", group="Linear OpMode")

public class EncoderTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor guideRailMotor = null;
    private DcMotor linearSlide = null;
    double ticks = 537.6;
    double newTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        guideRailMotor = hardwareMap.get(DcMotor.class, "guideRailMotor");
        linearSlide = hardwareMap.get(DcMotor.class, "liftMotor");

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        guideRailMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            if (gamepad1.a){
                encoderDivider(-5);
            }
            if (gamepad1.b){
                encoderDivider(5);
            }
            if (gamepad1.x){
                encoderMultiplier(-1);
            }
            if (gamepad1.y){
                encoderMultiplier(1);
            }
            if (linearSlide.getCurrentPosition() > 1000 || linearSlide.getCurrentPosition() > -1000){
                linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }

    }

    private void encoderDivider(int turnage) {
        guideRailMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks/turnage;
        guideRailMotor.setTargetPosition((int)newTarget);
        guideRailMotor.setPower(0.1);
        guideRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    private void encoderMultiplier(int turnage) {
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newTarget = ticks*turnage;
        guideRailMotor.setTargetPosition((int)newTarget);
        guideRailMotor.setPower(0.1);
        guideRailMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


}
