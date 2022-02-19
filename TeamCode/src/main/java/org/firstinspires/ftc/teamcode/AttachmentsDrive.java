//package org.firstinspires.ftc.teamcode;

//import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Timer;

//@TeleOp(name = "AttachmentsDrive", group = "TeleOp")

//public class AttachmentsDrive extends LinearOpMode {

    //DcMotor Intake;
    //DcMotor Slide;

    //double IntakePower;
    //double SlidePower;


 /*   public void init() {

        Intake = hardwareMap.dcMotor.get("Intake");
        Slide = hardwareMap.dcMotor.get("Slide");
    }

    public void loop() {


        //Intake.setPower(IntakePower);
        //Intake.setPower(-1);


        //Slide = gamepad2.left_stick_y;
        boolean inta = gamepad2.a;
        boolean intak = gamepad2.b;
        boolean rightBumper = gamepad2.right_bumper;

        /* Slide.setPower(SlidePower);
        Slide.setPower(-0.5);
        try {
            sleep(500);
        } catch (InterruptedException e) {
           // w
        }
        Slide.setPower(0.5);*/


        /* if (gamepad1.a) {
             Intake.setPower(1);

         } else {
             Intake.setPower(0);
         }

         if (gamepad1.b) {
             Slide.setPower(-1);

         } else if ("Encoder value" == 2050) {
             Slide.setPower(0);
         }

         if (gamepad1.dpad_down) {
             Slide.setPower(1);

         } else {
             Slide.setPower(0);
         }
     }

    @Override
    public void runOpMode() {

        Intake = hardwareMap.dcMotor.get("Intake");
        Slide = hardwareMap.dcMotor.get("Slide");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Encoder value", Slide.getCurrentPosition());
            telemetry.update();
        }
    }
}
                */
// Max encoder value for slide is ~2100

//if (inta) {
        //Intake.setPower(-1);

        //} else if (intak) {
        //Intake.setPower(0);
        //}
        //}}


   //}






