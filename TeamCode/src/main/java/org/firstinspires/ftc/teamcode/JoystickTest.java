
/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "JoystickTest", group = "TeleOp")

public class JoystickTest extends OpMode {

    Servo bucket;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Intake;
    DcMotor Slide;
    DcMotor Caro;


    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;


    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        Intake = hardwareMap.dcMotor.get("Intake");

        bucket = hardwareMap.servo.get("bucket");
        Slide = hardwareMap.dcMotor.get("Slide");
        Caro = hardwareMap.dcMotor.get("Caro");
    }

    @Override
    public void loop() {


        //Movement Controller

         //String boolean turn = String.valueOf(-gamepad1.right_stick_x,gamepad1.right_stick_x);





            right_drivePower = gamepad1.left_stick_y;
            back_left_drivePower = gamepad1.left_stick_y;
            left_drivePower = gamepad1.left_stick_y;
            back_right_drivePower = gamepad1.left_stick_y;



        right_drivePower = -gamepad1.right_stick_x;
        back_left_drivePower = -gamepad1.right_stick_x;
        left_drivePower = gamepad1.right_stick_x;
        back_right_drivePower = gamepad1.right_stick_x;


        left_drive.setPower(left_drivePower);
        right_drive.setPower(right_drivePower);
        back_left_drive.setPower(left_drivePower);
        back_right_drive.setPower(right_drivePower);

        //core drive
        boolean rightbumper = gamepad1.right_bumper;
        boolean leftbumper = gamepad1.left_bumper;
        //attachments
        boolean leftbumper1 = gamepad2.left_bumper;
        boolean rightbumper1 = gamepad2.right_bumper;

        //right_drivePower = -gamepad1.left_stick_x;
        //back_left_drivePower = -gamepad1.left_stick_x;
        //left_drivePower = gamepad1.left_stick_x;
        //back_right_drivePower = gamepad1.left_stick_x;

        //boolean rightBumper = gamepad2.right_bumper;
        if (leftbumper) {
            left_drive.setPower(1); // left drive is 0
            right_drive.setPower(-1); // right drive is 2
            back_left_drive.setPower(-1); // back left drive is 1
            back_right_drive.setPower(1); // back right drive is 3
        } else if (rightbumper) {
            left_drive.setPower(-1);
            right_drive.setPower(1);
            back_left_drive.setPower(1);
            back_right_drive.setPower(-1);
        }

        if (gamepad2.y) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }

        if(gamepad2.a) {
            Caro.setPower(-0.75); //3/4's speed (right carousal)
        } else if (gamepad2.b) {
            Caro.setPower(0.75); // (left carousal)
        } else{
            Caro.setPower(0);
        }



        if (rightbumper1) {
            Slide.setPower(-0.5);
        } else if (leftbumper1) {
            Slide.setPower(0.3);
        } else {
            Slide.setPower(0);
        }

        /*if (rightBumper) {
            Intake.setPower(-1);

        } else if (rightBumper) {
            Intake.setPower(0);
       }

        //servo motion has 270 degrees of motion; range is fro 0 to 1

        if (gamepad2.x) {
            bucket.setPosition(0.17);
        } else if (rightbumper1) {
            try {
                Thread.sleep(150);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            bucket.setPosition(.5);
        }else if (leftbumper1){
            bucket.setPosition(.85);


            telemetry.
            telemetry.update();
        }
    }}
    waitForStart();
        while (opModeIsActive()) {
                telemetry.addData("Encoder value", Slide.getCurerentPosition());
                telemetry.update(); */