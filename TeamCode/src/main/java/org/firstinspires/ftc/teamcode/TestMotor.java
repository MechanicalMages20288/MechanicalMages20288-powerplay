

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



/**
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Concept: Motor Test", group = "TeleOp")
//@Disabled
public class TestMotor extends OpMode {

    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;

    @Override
    public void init() {

        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            left_drive.setPower(1);

        } else if (gamepad1.b) {
            left_drive.setPower(-1);
        }
        else{
            left_drive.setPower(0);
        }
        if (gamepad1.y) {
            right_drive.setPower(1);
        } else if (gamepad1.x) {
            right_drive.setPower(-1);
        } else {
            right_drive.setPower(0);
        }
        if (gamepad2.b) {
            back_left_drive.setPower(-1);
        } else if (gamepad2.a) {
            back_left_drive.setPower(1);
        } else {
            back_left_drive.setPower(0);
        }
        if (gamepad2.y) {

            back_right_drive.setPower(1);
        } else if (gamepad2.x) {
            back_right_drive.setPower(-1);
        } else {
            back_right_drive.setPower(0);
        }
    }
}

