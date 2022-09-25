package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "MotorTest", group = "TeleOp")

public class MotorTest extends OpMode {

    DcMotor Vedaant;

    DcMotor Sidthesciencekid;


    @Override
    public void init() {

        Sidthesciencekid = hardwareMap.dcMotor.get("Intake");
        Vedaant= hardwareMap.dcMotor.get("Intake");

        //Intake = hardwareMap.dcMotor.get("Intake");
    }

    public void loop() {

        if (gamepad1.y) {
            Vedaant.setPower(0.5);
        } else  {
        Vedaant.setPower(0);
        }
       /* if(gamepad1.a){
            Intake.setPower(-1);
        } */
    }
}

