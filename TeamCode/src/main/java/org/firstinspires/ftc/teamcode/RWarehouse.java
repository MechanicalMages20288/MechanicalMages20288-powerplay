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



@Autonomous(name="RWarehouse", group = "auto")
public class RWarehouse extends LinearOpMode {

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
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);
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


// how many turns do the wheels need for 24 inches
// the distance you drive with one turn of the wheek is the circumference of the wheel
        double circumference6 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded6 = 17.427/circumference6;
        int encoderDrivingTarget6 = (int)(rotationsNeeded6*751);
        double circumference7 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded7 = -17.427/circumference7;
        int encoderDrivingTarget7 = (int)(rotationsNeeded7*751);


// set the target positions

        left_drive.setTargetPosition(encoderDrivingTarget7);
        right_drive.setTargetPosition(encoderDrivingTarget6);
        back_right_drive.setTargetPosition(encoderDrivingTarget7);
        back_left_drive.setTargetPosition(encoderDrivingTarget6);


// set the power for the motors

        left_drive.setPower(.4);
        right_drive.setPower(.4);
        back_right_drive.setPower(.4);
        back_left_drive.setPower(.4);


//set the motors to RUN_TO_POSITIO
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


// how many turns do the wheels need for 24 inches
// the distance you drive with one turn of the wheek is the circumference of the wheel
        double circumference = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded = 13/circumference;
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


//set the motors to RUN_TO_POSITIO
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
        bucket.setPosition(0.15);
        sleep(500);
        bucket.setPosition(.5);
        sleep(500);
        Slide.setPower(-0.5);
        sleep(1000);






// drive forward away from the carousel

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        double rotationsNeeded2 = -4/circumference;
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

//set the motors to RUN_TO_POSITIO

        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }
//wait for robot to execute its rout
//see what stage the robot is in

        telemetry.addData("Path", " going back");
        telemetry.update();



//stop robot

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);



        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double circumference3 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded3 = 16.2652/circumference3;
        int encoderDrivingTarget3 = (int)(rotationsNeeded3*751);

        double circumference4 = 3.14*3.70 ; // pi*diameter = circumference
        double rotationsNeeded4 = -16.2652/circumference4;
        int encoderDrivingTarget4 = (int)(rotationsNeeded4*751);

        left_drive.setTargetPosition(encoderDrivingTarget3);
        right_drive.setTargetPosition(encoderDrivingTarget4);
        back_right_drive.setTargetPosition(encoderDrivingTarget4);
        back_left_drive.setTargetPosition(encoderDrivingTarget3);


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




        drive_encoder(46.472, 0.75, "fwd");



        telemetry.addData("Path", "IN WAREHOUSE");
        telemetry.update();

        drive_encoder(6, 0.75, "fwd");
    }
    int drive_encoder(double distance, double speed, String direction ) {

        double circumference = 11.618;
        double rotationsNeeded = distance/circumference;
        int dir = 0;

        if (direction == "fwd") {
            dir = 1;
        }
        else if (direction == "bk"){
            dir = -1;
        }

        int encoderTarget = (int) (dir * rotationsNeeded*751);


        left_drive.setTargetPosition(encoderTarget);
        right_drive.setTargetPosition(encoderTarget);
        back_right_drive.setTargetPosition(encoderTarget);
        back_left_drive.setTargetPosition(encoderTarget);

        left_drive.setPower(speed);
        right_drive.setPower(speed);
        back_right_drive.setPower(speed);
        back_left_drive.setPower(speed);

        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);

        return(0);
    }
    int Turn_Or_Strafe_encoder(double distance, double lspeed, double rspeed , double brspeed , double blspeed ) {
        // To strafe left 1,-1,-1 , 1
        // To strafe right -1, 1, 1,-1
        // To turn right -1, 1,-1, 1
        // to turn left 1,-1, 1, -1

        double circumference = 11.618;
        double rotationsNeeded = distance/circumference;


        int lencoderTarget = (int) (lspeed * rotationsNeeded*751);
        int rencoderTarget = (int) (rspeed * rotationsNeeded*751);
        int blencoderTarget = (int) (blspeed * rotationsNeeded*751);
        int brencoderTarget = (int) (brspeed * rotationsNeeded*751);

        left_drive.setTargetPosition(lencoderTarget);
        right_drive.setTargetPosition(rencoderTarget);
        back_right_drive.setTargetPosition(brencoderTarget);
        back_left_drive.setTargetPosition(blencoderTarget);

        left_drive.setPower(lspeed);
        right_drive.setPower(rspeed);
        back_right_drive.setPower(brspeed);
        back_left_drive.setPower(blspeed);

        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (left_drive.isBusy() && back_right_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy()) {
        }

        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);

        return(0);
    }
}

