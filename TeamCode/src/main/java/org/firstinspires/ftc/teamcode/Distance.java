package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;




    /**
     * {@link SensorREV2mDistance} illustrates how to use the REV Robotics
     * Time-of-Flight Range Sensor.
     *
     * The op mode assumes that the range sensor is configured with a name of "sensor_range".
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
     *
     * @see <a href="http://revrobotics.com">REV Robotics Web Page</a>
     */
    @Autonomous(name = " Distance", group = "auto")

    public class Distance extends LinearOpMode {

        Servo bucket;
        DcMotor right_drive;
        DcMotor left_drive;
        DcMotor back_right_drive;
        DcMotor back_left_drive;
        DcMotor Intake;
        DcMotor Slide;
        DcMotor Caro;

        private DistanceSensor sensorRange;

        @Override
        public void runOpMode () {

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

            // you can use this as a regular DistanceSensor.
            sensorRange = hardwareMap.get(DistanceSensor.class, "distance");

            // you can also cast this to a Rev2mDistanceSensor if you want to use added
            // methods associated with the Rev2mDistanceSensor class.
            Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

            telemetry.addData(">>", "Press start to continue");
            telemetry.update();

            waitForStart();
            while(opModeIsActive()) {

                if ( sensorRange.getDistance(DistanceUnit.INCH) > 4.1){
                    left_drive.setPower(-.25);
                    right_drive.setPower(-.25);
                    back_left_drive.setPower(-.25);
                    back_right_drive.setPower(-.25);
                }
                else if ( sensorRange.getDistance(DistanceUnit.INCH) <=4.1 ) {
                    left_drive.setPower(0);
                    right_drive.setPower(0);
                    back_left_drive.setPower(0);
                    back_right_drive.setPower(0);

                    Caro.setPower(-0.75); //3/4's speed (right carousal)
                    sleep(5000);
                    Caro.setPower(0);
                }


                /* generic DistanceSensor methods.
                telemetry.addData("deviceName",sensorRange.getDeviceName() );
                telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
                telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER))); */
                telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));

                // Rev2mDistanceSensor specific methods.
                /* telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

                telemetry.update();*/
            }
        }

    }

