package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "IntakeDrive", group = "TeleOp")

public class IntakeDrive extends OpMode {

    DcMotor Intake;
    DcMotor Slide;

    double IntakePower;
    double SlidePower;

    public void init() {

        Intake = hardwareMap.dcMotor.get("Intake");
    }

    public void loop() {

        if (gamepad1.a) {
            Intake.setPower(1);
        } else {
            Intake.setPower(0);
        }







    }
}











