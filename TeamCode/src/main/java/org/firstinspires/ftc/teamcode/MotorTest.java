package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;



@TeleOp(name = "MotorTest", group = "TeleOp")

public class MotorTest extends OpMode {

    DcMotor Intake;
    double IntakePower;

    @Override
    public void init() {

        Intake = hardwareMap.dcMotor.get("Intake");
    }

    public void loop() {


        Intake.setPower(IntakePower);
        Intake.setPower(-1);
    }}

