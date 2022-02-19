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



@Autonomous(name="RedCaroAuto", group = "auto")
public class RedCaroAuto extends LinearOpMode {

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


//drive backward 2 feet = 24 inches
//reset encoders

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        double circumference6 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded6 = -29/circumference6;
        int encoderDrivingTarget6 = (int)(rotationsNeeded6*751);
        double circumference7 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded7 = 29/circumference7;
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
        sleep(500);
        Slide.setPower(-0.5);
        sleep(1000);
        Slide.setPower(0);

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
        double rotationsNeeded8 = 5 / circumference8;
        int encoderDrivingTarget8 = (int) (rotationsNeeded8 * 751);


// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget8);
        back_right_drive.setTargetPosition(encoderDrivingTarget8);
        right_drive.setTargetPosition(encoderDrivingTarget8);
        back_left_drive.setTargetPosition(encoderDrivingTarget8);


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





        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);


        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        double circumference15 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded15 = -15/circumference15;
        int encoderDrivingTarget15 = (int)(rotationsNeeded15*751);
        double circumference16 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded16 = 15/circumference16;
        int encoderDrivingTarget16 = (int)(rotationsNeeded16*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget16);
        right_drive.setTargetPosition(encoderDrivingTarget15);
        back_right_drive.setTargetPosition(encoderDrivingTarget15);
        back_left_drive.setTargetPosition(encoderDrivingTarget16);


// set the power for the motors

        left_drive.setPower(.3);
        right_drive.setPower(.3);
        back_right_drive.setPower(.3);
        back_left_drive.setPower(.3);


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


        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        double circumference20 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded20 = -2/circumference6;
        int encoderDrivingTarget20 = (int)(rotationsNeeded20*751);
        double circumference21 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded21 = 2/circumference21;
        int encoderDrivingTarget21 = (int)(rotationsNeeded21*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget20);
        right_drive.setTargetPosition(encoderDrivingTarget21);
        back_right_drive.setTargetPosition(encoderDrivingTarget20);
        back_left_drive.setTargetPosition(encoderDrivingTarget21);


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




        telemetry.addData("Path", "Driving to Carousel");
        telemetry.update();



        double circumference11 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded11 = -18/circumference11;
        int encoderDrivingTarget11 = (int)(rotationsNeeded11*751);



// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget11);
        right_drive.setTargetPosition(encoderDrivingTarget11);
        back_right_drive.setTargetPosition(encoderDrivingTarget11);
        back_left_drive.setTargetPosition(encoderDrivingTarget11);


// set the power for the motors

        left_drive.setPower(.3);
        right_drive.setPower(.3);
        back_left_drive.setPower(.3);
        back_right_drive.setPower(.3);



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

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference12 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded12 = -3.8/circumference12;
        int encoderDrivingTarget12 = (int)(rotationsNeeded12*751);



// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget12);
        right_drive.setTargetPosition(encoderDrivingTarget12);
        back_right_drive.setTargetPosition(encoderDrivingTarget12);
        back_left_drive.setTargetPosition(encoderDrivingTarget12);


// set the power for the motors

        left_drive.setPower(.15);
        right_drive.setPower(.15);
        back_left_drive.setPower(.15);
        back_right_drive.setPower(.15);



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
        Caro.setPower(-0.5); //3/4's speed (right carousal)
        sleep(5000);
        Caro.setPower(0);

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Path", "Driving to Storage Unit");
        telemetry.update();



        double circumference14 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded14 = 13/circumference14;
        int encoderDrivingTarget14 = (int)(rotationsNeeded14*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget14);
        right_drive.setTargetPosition(encoderDrivingTarget14);
        back_right_drive.setTargetPosition(encoderDrivingTarget14);
        back_left_drive.setTargetPosition(encoderDrivingTarget14);

// set the power for the motors

        left_drive.setPower(.4);
        right_drive.setPower(.3);
        back_right_drive.setPower(.35);
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



















































        /*
// how many turns do the wheels need for 24 inches
// the distance you drive with one turn of the wheel is the circumference of the wheel

        double circumference = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded = 15.5/circumference;     //change this for other auto mode too
        int encoderDrivingTarget = (int)(rotationsNeeded*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget);
        right_drive.setTargetPosition(encoderDrivingTarget);
        back_right_drive.setTargetPosition(encoderDrivingTarget);
        back_left_drive.setTargetPosition(encoderDrivingTarget);


// set the power for the motors

        left_drive.setPower(0.25);
        right_drive.setPower(0.25);
        back_right_drive.setPower(0.25);
        back_left_drive.setPower(0.25);


//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }
        telemetry.addData("Path", " going to shipping hub");
        telemetry.update();



//stop robot and deposit block into shipping hub (top level)

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);


        telemetry.addData("Path", " Depositing Block");
        telemetry.update();


        Slide.setPower(0.5);
        sleep(2000);
        bucket.setPosition(0.08);
        sleep(1300);
        bucket.setPosition(0.2);
        sleep(200);

// drive forward a little bit so that the bucket doesn't catch on the shipping hub  (1a)

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference1a = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded1a = -4/circumference1a;
        int encoderDrivingTarget1a = (int)(rotationsNeeded1a*751);


        left_drive.setTargetPosition(encoderDrivingTarget1a);
        right_drive.setTargetPosition(encoderDrivingTarget1a);
        back_right_drive.setTargetPosition(encoderDrivingTarget1a);
        back_left_drive.setTargetPosition(encoderDrivingTarget1a);


// set the power for the motors

        left_drive.setPower(0.25);
        right_drive.setPower(0.25);
        back_right_drive.setPower(0.25);
        back_left_drive.setPower(0.25);


//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }


        bucket.setPosition(.80);
        sleep(500);
        Slide.setPower(-0.5);
        sleep(1000);

// drive forward away from the carousel

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference2 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded2 = -8.2652/circumference2;
        int encoderDrivingTarget2 = (int)(rotationsNeeded2*751);



        left_drive.setTargetPosition(encoderDrivingTarget2);
        right_drive.setTargetPosition(encoderDrivingTarget2);
        back_right_drive.setTargetPosition(encoderDrivingTarget2);
        back_left_drive.setTargetPosition(encoderDrivingTarget2);


// set the power for the motors

        left_drive.setPower(0.25);
        right_drive.setPower(0.25);
        back_right_drive.setPower(0.25);
        back_left_drive.setPower(0.25);


//set the motors to RUN_TO_POSITION
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }
//turn 90 degrees left

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference3 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded3 = 15/circumference3;
        int encoderDrivingTarget3 = (int)(rotationsNeeded3*751);

        double circumference4 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded4 = -15/circumference4;
        int encoderDrivingTarget4 = (int)(rotationsNeeded4*751);

        left_drive.setTargetPosition(encoderDrivingTarget4);
        right_drive.setTargetPosition(encoderDrivingTarget3);
        back_right_drive.setTargetPosition(encoderDrivingTarget3);
        back_left_drive.setTargetPosition(encoderDrivingTarget4);


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


        telemetry.addData("Path", "turning");
        telemetry.update();

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        double circumference5 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded5 = -26.236/circumference5;
        int encoderDrivingTarget5 = (int)(rotationsNeeded5*751);



        left_drive.setTargetPosition(encoderDrivingTarget5);
        right_drive.setTargetPosition(encoderDrivingTarget5);
        back_right_drive.setTargetPosition(encoderDrivingTarget5);
        back_left_drive.setTargetPosition(encoderDrivingTarget5);




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


        telemetry.addData("Path", "move to carousel");
        telemetry.update();


        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        double circumference6 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded6 = 5/circumference6;
        int encoderDrivingTarget6 = (int)(rotationsNeeded6*751);

        double circumference7 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded7 = -5/circumference7;
        int encoderDrivingTarget7 = (int)(rotationsNeeded7*751);



        left_drive.setTargetPosition(encoderDrivingTarget6);
        right_drive.setTargetPosition(encoderDrivingTarget7);
        back_right_drive.setTargetPosition(encoderDrivingTarget7);
        back_left_drive.setTargetPosition(encoderDrivingTarget6);




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



        Caro.setPower(-0.75);
        sleep(5000);
        Caro.setPower(0);


         }

    }

*/


