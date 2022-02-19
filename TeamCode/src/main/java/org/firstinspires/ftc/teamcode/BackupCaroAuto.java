package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.transition.Slide;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name="BCaroAuto", group = "auto")
public class BackupCaroAuto extends LinearOpMode {

    static final int MOTOR_TICK_COUNTS = 751;
//inizitialize


    Servo bucket;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Intake;
    DcMotor Slide;
    DcMotor Caro;





    @Override
    public void runOpMode () throws InterruptedException {
        // hardware map


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


        waitForStart();

        bucket.setPosition(.5);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        double circumference6 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded6 = 29/circumference6;
        int encoderDrivingTarget6 = (int)(rotationsNeeded6*751);
        double circumference7 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded7 = -29/circumference7;
        int encoderDrivingTarget7 = (int)(rotationsNeeded7*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget6);
        right_drive.setTargetPosition(encoderDrivingTarget7);
        back_right_drive.setTargetPosition(encoderDrivingTarget6);
        back_left_drive.setTargetPosition(encoderDrivingTarget7);


// set the power for the motors

        left_drive.setPower(.4);
        right_drive.setPower(.4);
        back_right_drive.setPower(.4);
        back_left_drive.setPower(.4);


//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }
//wait for robot to execute its rout
//see what stage the robot is in
        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);


        telemetry.addData("Path", "Moving to Shipping Hub");
        telemetry.update();

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        double circumference10 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded10 = 4.5 / circumference10;
        int encoderDrivingTarget10 = (int) (rotationsNeeded10 * 751);


        left_drive.setTargetPosition(encoderDrivingTarget10);
        right_drive.setTargetPosition(encoderDrivingTarget10);
        back_right_drive.setTargetPosition(encoderDrivingTarget10);
        back_left_drive.setTargetPosition(encoderDrivingTarget10);


        left_drive.setPower(0.25);
        right_drive.setPower(0.25);
        back_right_drive.setPower(0.25);
        back_left_drive.setPower(0.25);


        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Path", "STRAFE TO THE LEFT IN LINE WITH THE CAROUSEL");
        telemetry.update();

        //STRAFE TO THE LEFT IN LINE WITH THE SHIPPING HUB
// how many turns do the wheels need for 24 inches
// the distance you drive with one turn of the wheel is the circumference of the wheel


        telemetry.addData("Path", " Depositing Block");
        telemetry.update();

        Slide.setPower(0.5);
        sleep(2000);
        bucket.setPosition(0.08);
        sleep(1300);
        bucket.setPosition(.5);
        sleep(1300);
        bucket.setPosition(0.08);
        sleep(1300);
        bucket.setPosition(.5);


        // how many turns do the wheels need for 24 inches
// the distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference3 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded3 = -22 / circumference3;
        int encoderDrivingTarget3 = (int) (rotationsNeeded3 * 751);


// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget3);
        back_right_drive.setTargetPosition(encoderDrivingTarget3);
        right_drive.setTargetPosition(encoderDrivingTarget3);
        back_left_drive.setTargetPosition(encoderDrivingTarget3);


// set the power for the motors

        left_drive.setPower(0.25);
        back_right_drive.setPower(0.25);
        right_drive.setPower(0.25);
        back_left_drive.setPower(0.25);

//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }
//wait for robot to execute its route

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference8 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded8 = 4 / circumference8;
        int encoderDrivingTarget8 = (int) (rotationsNeeded8 * 751);


// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget8);
        back_right_drive.setTargetPosition(encoderDrivingTarget8);
        right_drive.setTargetPosition(encoderDrivingTarget8);
        back_left_drive.setTargetPosition(encoderDrivingTarget8);


// set the power for the motors

        left_drive.setPower(0.27);
        back_right_drive.setPower(0.27);
        right_drive.setPower(0.27);
        back_left_drive.setPower(0.27);

//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }



        sleep(500);
        Slide.setPower(-0.5);
        sleep(1000);
        Slide.setPower(0);


        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);


        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




        telemetry.addData("Path", "Driving to Carousel");
        telemetry.update();



        double circumference11 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded11 = 24.5/circumference11;
        int encoderDrivingTarget11 = (int)(rotationsNeeded11*751);
        double circumference12 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded12 = -24.5/circumference12;
        int encoderDrivingTarget12 = (int)(rotationsNeeded12*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget12);
        right_drive.setTargetPosition(encoderDrivingTarget11);
        back_right_drive.setTargetPosition(encoderDrivingTarget12);
        back_left_drive.setTargetPosition(encoderDrivingTarget11);


// set the power for the motors

        left_drive.setPower(.15);
        right_drive.setPower(.15);
        back_right_drive.setPower(.15);
        back_left_drive.setPower(.15);


//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }
//wait for robot to execute its route


//stop robot

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);


//drive forward 2 feet = 24 inches
//reset encoders
        Caro.setPower(0.5); //3/4's speed (right carousal)
        sleep(5000);
        Caro.setPower(0);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Path", "Driving to Storage Unit");
        telemetry.update();


        double circumference13 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded13 = 12.5/circumference13;
        int encoderDrivingTarget13 = (int)(rotationsNeeded13*751);
        double circumference14 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded14 = -12.5/circumference14;
        int encoderDrivingTarget14 = (int)(rotationsNeeded14*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget13);
        right_drive.setTargetPosition(encoderDrivingTarget14);
        back_right_drive.setTargetPosition(encoderDrivingTarget13);
        back_left_drive.setTargetPosition(encoderDrivingTarget14);

// set the power for the motors

        left_drive.setPower(.4);
        right_drive.setPower(.4);
        back_right_drive.setPower(.4);
        back_left_drive.setPower(.4);


//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }

        telemetry.addData("Path", "Parked");
        telemetry.update();



//stop robot


    }

}

