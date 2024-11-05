package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo TEST")
@Disabled
public class ServoTest extends LinearOpMode {
    private Servo Wrist;

    @Override
    public void runOpMode() {
        Wrist = hardwareMap.get(Servo.class, "Wrist");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.left_bumper){
                Wrist.setPosition(0.5);
            }
        }
    }
}
