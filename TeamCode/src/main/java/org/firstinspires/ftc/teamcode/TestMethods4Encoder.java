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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="TestMethods4Encoder", group = "Auto")

public class TestMethods4Encoder extends LinearOpMode {

    static final int MOTOR_TICK_COUNTS = 751;

    Servo bucket;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Intake;
    DcMotor Slide;
    DcMotor Caro;

    BNO055IMU imu;

    Orientation angles;


    static final double COUNTS_PER_MOTOR_GOBILDA = 751;    // eg: GOBILDA MOTOR ENCODER
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.77953;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_GOBILDA * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable

    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;



    @Override
    public void runOpMode() throws InterruptedException {


        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        Intake = hardwareMap.dcMotor.get("Intake");
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);
        bucket = hardwareMap.servo.get("bucket");
        Slide = hardwareMap.dcMotor.get("Slide");
        Caro = hardwareMap.dcMotor.get("Caro");

        //set parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNUO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();




        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading", angles.firstAngle);
            telemetry.addData("Roll", angles.secondAngle);
            telemetry.addData("Pitch", angles.thirdAngle);
            telemetry.update();



        }





        // Add these to See where the robot is
        /** telemetry.addData("Path","Done With FWD");
         telemetry.addData("Path","Done with BK");
         telemetry.addData("Path","Done With Strafe");
         telemetry.update(); **/

/*
        Drive_encoder(23.236, .15, "fwd");

        Strafe_Encoder(23.236, .25, .25, "Right");

        Strafe_Encoder(23, .25, .25, "Left");

        Drive_encoder(23.236, .15, "bk");

        Turn_Encoder(15, .25, .25, "Right");

        Turn_Encoder(15, .25, .25, "Left");


        telemetry.addData("Ldrive value", left_drive.getCurrentPosition());
        telemetry.addData("BLdrive value", back_left_drive.getCurrentPosition());
        telemetry.addData("Rdrive value", right_drive.getCurrentPosition());
        telemetry.addData("Brdrive value", back_right_drive.getCurrentPosition());
        telemetry.update();

*/
    }

    int Drive_encoder(double distance, double speed, String direction) {

        double circumference = 11.618;
        double rotationsNeeded = distance / circumference;
        int dir = 0;

        if (direction == "fwd") {
            dir = -1;
        } else if (direction == "bk") {
            dir = 1;
        }

        int encoderTarget = (int) (dir * rotationsNeeded * 751);


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

        telemetry.addData("Path", "Done With FWD/BK");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return (0);
    }

    int Strafe_Encoder(double distance, double L_And_Br, double R_And_BL, String direction) {
        // To strafe left 1,-1,-1 , 1
        // To strafe right -1, 1, 1,-1
        // To turn right -1, 1,-1, 1
        // to turn left 1,-1, 1, -1

        int R_Bldir = 0;
        int L_BRdir = 0;

        if (direction == "Left") {
            R_Bldir = -1;
            L_BRdir = 1;
        } else if (direction == "Right") {
            R_Bldir = 1;
            L_BRdir = -1;
        }

        double circumference = 11.618;
        double rotationsNeededR_And_BL = distance / circumference;
        int encoderDrivingTargetR_And_Bl = (int) (R_Bldir * rotationsNeededR_And_BL * 751);

        double rotationsNeededL_And_BR = distance / circumference;
        int encoderDrivingTargetL_And_BR = (int) (L_BRdir * rotationsNeededL_And_BR * 751);


        left_drive.setTargetPosition(encoderDrivingTargetL_And_BR);
        right_drive.setTargetPosition(encoderDrivingTargetR_And_Bl);
        back_right_drive.setTargetPosition(encoderDrivingTargetL_And_BR);
        back_left_drive.setTargetPosition(encoderDrivingTargetR_And_Bl);

        left_drive.setPower(L_And_Br);
        right_drive.setPower(R_And_BL);
        back_right_drive.setPower(L_And_Br);
        back_left_drive.setPower(R_And_BL);

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
        telemetry.addData("Path", "Done With Strafe");

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return (0);
    }

  /*  int Turn_Encoder(double distance, double L_And_BL, double R_And_BR, String direction) {
        // To strafe left 1,-1,-1 , 1
        // To strafe right -1, 1, 1,-1
        // To turn right -1, 1,-1, 1
        // to turn left 1,-1, 1, -1

        int Rightdir = 0;
        int Leftdir = 0;

        if (direction == "Left") {
            Rightdir = -1;
            Leftdir = 1;
        } else if (direction == "Right") {
            Rightdir = 1;
            Leftdir = -1;
        }


        double circumference = 11.618;

        double rotationsNeededR_And_BR = distance / circumference;
        int encoderDrivingTargetR_And_BR = (int) (Rightdir * rotationsNeededR_And_BR * 751);

        double rotationsNeededL_And_BL = distance / circumference;
        int encoderDrivingTargetL_And_BL = (int) (Leftdir * rotationsNeededL_And_BL * 751);


        left_drive.setTargetPosition(encoderDrivingTargetL_And_BL);
        right_drive.setTargetPosition(encoderDrivingTargetR_And_BR);
        back_right_drive.setTargetPosition(encoderDrivingTargetR_And_BR);
        back_left_drive.setTargetPosition(encoderDrivingTargetL_And_BL);

        left_drive.setPower(L_And_BL);
        right_drive.setPower(R_And_BR);
        back_right_drive.setPower(R_And_BR);
        back_left_drive.setPower(L_And_BL);

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
        telemetry.addData("Path", "Done With Turn");

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return (0);
    }*/


}