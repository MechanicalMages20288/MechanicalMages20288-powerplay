package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="ColorSense", group = "auto")
public class ColorTest extends LinearOpMode {


ColorSensor color;

     @Override
     public void runOpMode() throws InterruptedException {

         color = hardwareMap.get(ColorSensor.class, "color");



         waitForStart();
         while(opModeIsActive() && !isStopRequested()){

             if (color.red() > 30 ){
                 telemetry.addData("Path", "Haha");
             }
             else {
                 telemetry.addData("Path", "nope");
             }

             telemetry.addData("red: ", color.red()); //checking for colors
             telemetry.addData("green: ", color.green()); //checking for colors
             telemetry.addData("blue: ", color.blue()); //checking for colors
             telemetry.addData("alpha: ",color.alpha()); //the amount of light is alpha
             telemetry.update();


         }



     }
 }
