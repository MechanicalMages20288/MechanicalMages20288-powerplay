
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "BlueMecanum", group = "TeleOp")

public class TestMecanum extends OpMode {

    static final int MOTOR_TICK_COUNTS = 751;


    Servo bucket;
    Servo Vcap;
    CRServo Hcap;
    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Intake;
    DcMotor Slide;
    DcMotor Caro;

    //Capping
    double current_down_pos;

    //Motor Power
    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;
    double IntakePower;

    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        Intake = hardwareMap.dcMotor.get("Intake");

        Vcap = hardwareMap.servo.get("Vcap");
        Hcap = hardwareMap.crservo.get("Hcap");
        bucket = hardwareMap.servo.get("bucket");
        Slide = hardwareMap.dcMotor.get("Slide");
        Caro = hardwareMap.dcMotor.get("Caro");

        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    @Override
    public void loop() {

        //Movement Controller
        right_drivePower = gamepad1.right_stick_y * -1.0;
        back_left_drivePower = gamepad1.right_stick_y * -1.0;
        left_drivePower = gamepad1.left_stick_y * -1.0;
        back_right_drivePower = gamepad1.left_stick_y * -1.0;

        left_drive.setPower(left_drivePower);
        right_drive.setPower(right_drivePower);
        back_left_drive.setPower(left_drivePower);
        back_right_drive.setPower(right_drivePower);

        boolean rightbumper = gamepad1.right_bumper; //Strafe Right
        boolean leftbumper = gamepad1.left_bumper; //Strafe Left
        //attachments
        boolean down = gamepad2.left_bumper;
        boolean up = gamepad2.right_bumper;

        boolean Upcap = gamepad2.dpad_up;
        boolean Downcap = gamepad2.dpad_down;
        boolean Rightcap = gamepad2.dpad_right;
        boolean Leftcap = gamepad2.dpad_left;

        Intake.setPower(IntakePower);

        IntakePower = gamepad2.right_trigger;

        /*if(gamepad1.b){
            hasChanged = lastState != B

            ToggleState;
            isJustPressed = BToggleState && hasChanged;

        }
        if(isJustPressed){
            isToggled = !isToggled;
        }

           if (gamepad1.b) {
            currPressed = true;
        } else {
            currPressed = false;
        }

        if (currPressed &&  currPressed != lastPressed) {
            WallToggle = true;  //if user presses a B their first time
        } else if (WallToggle) {
            //since this an if-else, it is able to assume that currPressed is true
            WallToggle = false; //toggle is already true but driver presses B
        }
        lastPressed = currPressed; */

        // Wall Hugger
        if (gamepad1.dpad_up) {
            left_drive.setPower(.95);
            right_drive.setPower(1);
            back_left_drive.setPower(.95);
            back_right_drive.setPower(1);
        } else if (gamepad1.dpad_down) {
            left_drive.setPower(-.95);
            right_drive.setPower(-1);
            back_left_drive.setPower(-.95);
            back_right_drive.setPower(-1);
        } else {
            left_drive.setPower(left_drivePower);
            right_drive.setPower(right_drivePower);
            back_left_drive.setPower(left_drivePower);
            back_right_drive.setPower(right_drivePower);
        }

        // Precision moves
        if(gamepad1.x){
            left_drive.setPower(-.23);
            right_drive.setPower(-.23);
            back_left_drive.setPower(-.23);
            back_right_drive.setPower(-.23);
        }

        if(gamepad1.a){
            left_drive.setPower(-.4);
            right_drive.setPower(-.4);
            back_left_drive.setPower(-.4);
            back_right_drive.setPower(-.4);
        }

        //boolean rightBumper = gamepad2.right_bumper;
        //Strafing
        if (rightbumper) {
            left_drive.setPower(1); // left drive is 0
            right_drive.setPower(-1); // right drive is 2
            back_left_drive.setPower(-1); // back left drive is 1
            back_right_drive.setPower(1); // back right drive is 3
        } else if (leftbumper) {
            left_drive.setPower(-1);
            right_drive.setPower(1);
            back_left_drive.setPower(1);
            back_right_drive.setPower(-1);
        }

        // Is this duplicate with wall hugger
        /*if (gamepad1.dpad_up) {
            left_drive.setPower(0.95);
            right_drive.setPower(1);
            back_left_drive.setPower(0.95);
            back_right_drive.setPower(1);
        } else if (gamepad1.dpad_down) {
            left_drive.setPower(-0.95);
            right_drive.setPower(-1);
            back_left_drive.setPower(-0.95);
            back_right_drive.setPower(-1);
        } else {
            left_drive.setPower(left_drivePower);
            right_drive.setPower(right_drivePower);
            back_left_drive.setPower(left_drivePower);
            back_right_drive.setPower(right_drivePower);
        }*/

        // go forward slowly
        if(gamepad1.y){
            left_drive.setPower(.2);
            right_drive.setPower(.2);
            back_left_drive.setPower(.2);
            back_right_drive.setPower(.2);
        }

        current_down_pos = Vcap.getPosition();

        if (Downcap) {
            telemetry.addData("Servo Vertical", Vcap.getPosition());
            telemetry.update();
            Vcap.setPosition(current_down_pos + 0.0005);

        }
        if (Upcap) {
            telemetry.addData("Servo Vertical", Vcap.getPosition());
            telemetry.update();
            Vcap.setPosition(current_down_pos - 0.0005);

        }
        if (Leftcap) {
            Hcap.setPower(-.15);
        }
        else if (Rightcap){
            Hcap.setPower(.15);
        }
        else {
            Hcap.setPower(0);
        }

        if (gamepad2.y) {
            Intake.setPower(-1);
        } else {
            Intake.setPower(0);
        }

        if (gamepad2.a) {
            Caro.setPower(Math.min(Caro.getPower() - 0.0075, -.75));
        } else if (gamepad2.b) {
            Caro.setPower(Math.max(Caro.getPower() + 0.0075, .75));
        } else {
            Caro.setPower(0);
        }

        telemetry.addData("Encoder value", Slide.getCurrentPosition());
        telemetry.update();

        if (up && Slide.getCurrentPosition() >= -1800) {
            Slide.setPower(-.55);
            telemetry.addData("Slide", Slide.getCurrentPosition());
            telemetry.update();

        } else if (down) {

            Slide.setPower(.4);
            telemetry.addData("Slide", Slide.getCurrentPosition());
            telemetry.update();

        } else {
            Slide.setPower(0);

        }

        if (up && Slide.getCurrentPosition() <= -330) {

            bucket.setPosition(0.4);
        } else if (down) {
            bucket.setPosition(.78);
        }
        if (gamepad2.x) {
            bucket.setPosition(0.05);
        }
    }


}




