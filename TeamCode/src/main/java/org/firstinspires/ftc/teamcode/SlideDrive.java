package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SlideDrive", group = "TeleOp")

public class SlideDrive extends OpMode {

    DcMotor Slide;

    double SlidePower;

    public void init() {

        Slide = hardwareMap.dcMotor.get("Slide");
    }

    public void loop() {

        if (gamepad1.a) {
            Slide.setPower(-0.5);
        } else if (gamepad1.b) {
            Slide.setPower(0.3);
        } else {
            Slide.setPower(0);
        }









    }
}











