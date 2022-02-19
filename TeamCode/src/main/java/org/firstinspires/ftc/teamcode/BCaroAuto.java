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



@Autonomous(name="BackupCaroAuto", group = "auto")
public class BCaroAuto extends LinearOpMode {

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


        double circumference10 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded10 = 4 / circumference10;
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

        telemetry.addData("Path", "STRAFE TO THE LEFT IN LINE WITH THE SHIPPING HUB");
        telemetry.update();

        //STRAFE TO THE LEFT IN LINE WITH THE SHIPPING HUB
// how many turns do the wheels need for 24 inches
// the distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference6 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded6 = -20 / circumference6;
        int encoderDrivingTarget6 = (int) (rotationsNeeded6 * 751);
        double circumference7 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded7 = 20 / circumference7;
        int encoderDrivingTarget7 = (int) (rotationsNeeded7 * 751);

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


//set the motors to RUN_TO_POSITIOn
//
//
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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


        telemetry.addData("Path", "Moving to Shipping Hub");
        telemetry.update();


        // how many turns do the wheels need for 24 inches
// the distance you drive with one turn of the wheel is the circumference of the wheel
        double circumference3 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded3 = 10 / circumference3;
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


        telemetry.addData("Path", " Depositing Block");
        telemetry.update();


        Slide.setPower(0.5);
        sleep(2000);
        bucket.setPosition(0.08);
        sleep(1300);
        bucket.setPosition(0.15);
        sleep(200);
        bucket.setPosition(.5);
        sleep(1300);
        bucket.setPosition(0.08);
        sleep(1000);
        bucket.setPosition(.5);
        sleep(500);
        Slide.setPower(-0.5);
        sleep(1000);
        Slide.setPower(0);

        double circumference5 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded5 = -4 / circumference5;
        int encoderDrivingTarget5 = (int) (rotationsNeeded5 * 751);


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


        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);





        double circumference8 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded8 = 15.1034/circumference8;
        int encoderDrivingTarget8 = (int)(rotationsNeeded8*751);
        double circumference9 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded9 = -15.1034/circumference9;
        int encoderDrivingTarget9 = (int)(rotationsNeeded9*751);

// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget8);
        right_drive.setTargetPosition(encoderDrivingTarget9);
        back_right_drive.setTargetPosition(encoderDrivingTarget8);
        back_left_drive.setTargetPosition(encoderDrivingTarget9);


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

        telemetry.addData("Path", "Driving 2 feet");
        telemetry.update();



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

        double circumference16 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded16 = -4 / circumference16;
        int encoderDrivingTarget16 = (int) (rotationsNeeded16 * 751);


        left_drive.setTargetPosition(encoderDrivingTarget16);
        right_drive.setTargetPosition(encoderDrivingTarget16);
        back_right_drive.setTargetPosition(encoderDrivingTarget16);
        back_left_drive.setTargetPosition(encoderDrivingTarget16);


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

        double circumference25 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded25 = -11 / circumference25;
        int encoderDrivingTarget25 = (int) (rotationsNeeded25 * 751);


        left_drive.setTargetPosition(encoderDrivingTarget25);
        right_drive.setTargetPosition(encoderDrivingTarget25);
        back_right_drive.setTargetPosition(encoderDrivingTarget25);
        back_left_drive.setTargetPosition(encoderDrivingTarget25);


        left_drive.setPower(-0.25);
        right_drive.setPower(0.25);
        back_right_drive.setPower(0.25);
        back_left_drive.setPower(-0.25);


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


        double circumference51 = 3.14 * 3.70; // pi*diameter = circumference
        double rotationsNeeded51 = 11 / circumference51;
        int encoderDrivingTarget51 = (int) (rotationsNeeded51 * 751);


        left_drive.setTargetPosition(encoderDrivingTarget51);
        right_drive.setTargetPosition(encoderDrivingTarget51);
        back_right_drive.setTargetPosition(encoderDrivingTarget51);
        back_left_drive.setTargetPosition(encoderDrivingTarget51);


        left_drive.setPower(-0.25);
        right_drive.setPower(0.25);
        back_right_drive.setPower(0.25);
        back_left_drive.setPower(-0.25);


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




//wait for robot to execute its rout
//see what stage the robot is in

        telemetry.addData("Path", "Parking in Storage Unit");
        telemetry.update();



//stop robot

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);




    }

}