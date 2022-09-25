package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


public class Sensor_Methods_Test_Auto extends LinearOpMode {

    private DistanceSensor sensorRange;
    ColorSensor color;
    Servo bucket;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Intake;
    DcMotor Slide;
    DcMotor Caro;


    @Override
    public void runOpMode() throws InterruptedException {
        color = hardwareMap.get(ColorSensor.class, "color");
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
        sensorRange = hardwareMap.get(DistanceSensor.class, "distance");


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {


        drive_encoder(29.045, .5, "fwd");

        /*if(color.red() > 30) {
            drive_encoder(6,.5,"fwd");
        }*/







            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            telemetry.addData("red: ", color.red()); //checking for colors
            telemetry.addData("green: ", color.green()); //checking for colors
            telemetry.addData("blue: ", color.blue()); //checking for colors
            telemetry.addData("alpha:/light ", color.alpha()); //the amount of light is alpha
            telemetry.update();

        }


    }
    int drive_encoder(double distance, double speed, String direction ) {

        double circumference = 11.618;
        double rotationsNeeded = distance/circumference;
        int dir = 0;

        if (direction == "fwd") {
            dir = -1;
        }
        else if (direction == "bk"){
            dir = 1;
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

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return(0);
    }
    int Turn_Or_Strafe_encoder(double distance, double lspeed, double rspeed , double brspeed , double blspeed ) {
        // To strafe left 1,-1,-1 , 1
        // To strafe right -1, 1, 1,-1
        // To turn right -1, 1,-1, 1
        // to turn left 1,-1, 1, -1

        double circumference = 11.618;
        double rotationsNeeded = distance/circumference;



        int lencoderTarget = (int) (rotationsNeeded*751);
        int rencoderTarget = (int) ( rotationsNeeded*751);
        int blencoderTarget = (int) (rotationsNeeded*751);
        int brencoderTarget = (int) ( rotationsNeeded*751);

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

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return(0);
    }
}