package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*@Autonomous(name="Auto")
public class Auto<right_drivePos, back_right_drivePos, left_drivePos, back_left_drivePos> extends LinearOpMode  {

    Servo tpa;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Intake;
    DcMotor Slide;



    private int right_drivePos;
    private int back_right_drivePos;
    private int left_drivePos;
    private int back_left_drivePos;




    @Override
    public void runOpMode() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        Intake = hardwareMap.dcMotor.get("Intake");

        tpa = hardwareMap.servo.get("tpa");
        Slide = hardwareMap.dcMotor.get("Slide");


        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_right_drivePos = 0;
        left_drivePos = 0;
        back_left_drivePos = 0;
        right_drivePos = 0;

        waitForStart();

        

        private void Target (int left_driveTarget;  int right_driveTarget; int back_right_driveTarget; int back_left_driveTarget; double speed;);
        {

            drive(1000, 1000, 1000, 1000);

            back_right_drivePos += back_right_driveTarget;
            left_drivePos += left_driveTarget;
            back_left_drivePos += back_left_driveTarget;
            right_drivePos += right_driveTarget;

            back_left_drive.setTargetPosition(back_left_drivePos);
            right_drive.setTargetPosition(right_drivePos);
            back_right_drive.setTargetPosition(back_right_drivePos);
            left_drive.setTargetPosition(left_drivePos);

            back_left_drive.setPower(speed);
            right_drive.setPower(speed);
            left_drive.setPower(speed);
            back_right_drive.setPower(speed);

        }

        while(opModeIsActive() && left_drive.isBusy() && right_drive.isBusy() &&back_right_drive.isBusy() &&back_left_drive.isBusy() ); {

        }


    }

    private void drive(int i, int i1, int i2, int i3) {
    }

}*/
