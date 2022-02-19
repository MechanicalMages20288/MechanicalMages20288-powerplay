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
import com.qualcomm.robotcore.util.ElapsedTime;

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
    DcMotor caro;


    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;




    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()){

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.addData("Roll", angles.secondAngle);
        telemetry.addData("Pitch", angles.thirdAngle);
        telemetry.update();

        }



    }
}
