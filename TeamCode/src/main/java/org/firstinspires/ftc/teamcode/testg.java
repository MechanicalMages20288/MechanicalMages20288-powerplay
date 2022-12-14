package org.firstinspires.ftc.teamcode;

import android.transition.Slide;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="testg", group = "auto")
public class testg extends LinearOpMode {


    BNO055IMU imu;


    Orientation angles;


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



    private Orientation lastAngles = new Orientation();

    private double currAngle = 0.0;
    private ElapsedTime     runtime = new ElapsedTime();




    @Override
    public void runOpMode() throws InterruptedException {
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


        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);
        Intake = hardwareMap.dcMotor.get("Intake");


        Caro = hardwareMap.dcMotor.get("Caro");
        bucket = hardwareMap.servo.get("bucket");
        Slide = hardwareMap.dcMotor.get("Slide");



        waitForStart();

        Drive_encoder(20,.3,"fwd");
        Drive_encoder(20,.3,"fwd");


        right_turn(90);
        sleep(500);
        left_turn(-90);
    }

    // resets currAngle Value
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }

    public double getAngle() {

        // Get current orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }

    public void right_turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error)  >= 2) {
            double motorPower = (error < 0 ? -0.2: 0.2);
            left_drive.setPower(motorPower);
            right_drive.setPower(-motorPower);
            back_left_drive.setPower(motorPower);
            back_right_drive.setPower(-motorPower);
            error = degrees - (getAngle());
            telemetry.addData("error", error);
            telemetry.update();
        }

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);
    }

    public void left_turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error)  >= 2) {
            double motorPower = (error < 0 ? -0.2: 0.2);
            left_drive.setPower(motorPower);
            right_drive.setPower(-motorPower);
            back_left_drive.setPower(motorPower);
            back_right_drive.setPower(-motorPower);
            error = degrees - Math.abs(getAngle());
            telemetry.addData("error", error);
            telemetry.update();
        }

        left_drive.setPower(0);
        right_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);
    }


    public void turnTo(double degrees){

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        System.out.println(orientation.firstAngle);
        double error = degrees - orientation.firstAngle;

        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }

        left_turn(error);
    }

    public double getAbsoluteAngle() {
        return imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (Math.abs(targetAngle - getAbsoluteAngle()) > 0.5 || pid.getLastSlope() > 0.75) {
            double motorPower = pid.update(getAbsoluteAngle());
            left_drive.setPower(-motorPower);
            right_drive.setPower(motorPower);
            back_left_drive.setPower(-motorPower);
            back_right_drive.setPower(motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        left_drive.setPower(0);
        right_drive.setPower(0);
        back_left_drive.setPower(0);
        back_right_drive.setPower(0);
    }
    int Drive_encoder(double distance, double speed, String direction ) {

        telemetry.addData("Motors","Test");
        telemetry.update();
        sleep(5000);

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

        telemetry.addData("Path","Done With FWD/BK");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return(0);
    }
    int Strafe_Encoder(double distance, double L_And_Br , double R_And_BL, String direction) {
        // To strafe left 1,-1,-1 , 1
        // To strafe right -1, 1, 1,-1
        // To turn right -1, 1,-1, 1
        // to turn left 1,-1, 1, -1

        int R_Bldir = 0;
        int L_BRdir = 0;

        if (direction == "Left") {
            R_Bldir = -1;
            L_BRdir = 1;
        }
        else if (direction == "Right"){
            R_Bldir = 1;
            L_BRdir = -1;
        }

        double circumference = 11.618;
        double rotationsNeededR_And_BL = distance/circumference;
        int encoderDrivingTargetR_And_Bl = (int)(R_Bldir * rotationsNeededR_And_BL*751);

        double rotationsNeededL_And_BR = distance/circumference;
        int encoderDrivingTargetL_And_BR = (int)(L_BRdir * rotationsNeededL_And_BR*751);



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
        telemetry.addData("Path","Done With Strafe");

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return(0);
    }
}