package org.firstinspires.ftc.teamcode.MechMagesAuto;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;




@Autonomous(name = "Red Caro Auto", group = "Auto")

public class RedCaroAuto extends LinearOpMode {


    Servo Vcap;
    Servo bucket;

    DcMotor Slide;
    DcMotor Caro;




    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "TSE_20388.tflite";
    private static final String[] LABELS = {
            //"Ball",
            //"Cube",
            //"Duck",
            //"Marker"
            "TSE"
    };


    private static final String VUFORIA_KEY = "AfNMxFT/////AAABmR7zRcVeVkqKvU4N1c7e1oxF1BA0BsMo8KWcNj9QcX7YkXRTzYnTB5CXxuwZrCakOrNCNJug1QsLFhH1H3NmJjt0+OunpWkTNIkfDvoIJGyPRnq44a7JWMUrsbLgMjBfluuMcg7o10Jiq/NO6ydSWSvip8MKecAnam1hA5I5k9Ja0L82NUF6NdRERl7yt8p7v68c04phX1Ijnl2ngfNdXQ7iXzB1WTNAfrQGQ/h5b1NJNTC+3mmPA8f80X0blEHcqzZ6YxW5aW29oE7obu3OJuOXKOuQB0uzc9mNS9xriR5ZDQfadVPnc+V7zsOJH9owbALKgQmYwm6Z4bI8S90A82TcQusVbOrbUuCD0k3WHYVZ";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;



    @Override
    public void runOpMode()throws InterruptedException {
        initVuforia();
        initTfod();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Caro = hardwareMap.dcMotor.get("Caro");
        Slide = hardwareMap.dcMotor.get("Slide");
        Vcap =hardwareMap.servo.get("Vcap");
        bucket = hardwareMap.servo.get("bucket");

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1Pos3 = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .waitSeconds(.1)
                .strafeRight(30)
                .build();
        TrajectorySequence trajSeq2Pos3 = drive.trajectorySequenceBuilder(trajSeq1Pos3.end())
                .lineToLinearHeading(new Pose2d(-51,-15, Math.toRadians(-90)))
                .waitSeconds(.2)
                .back(14)
                .build();
        TrajectorySequence trajSeq3Pos3 = drive.trajectorySequenceBuilder(trajSeq2Pos3.end())
                .forward(32)
                .waitSeconds(.1)
                .strafeLeft(15)
                .build();

        TrajectorySequence trajSeq1Pos2 = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .waitSeconds(.1)
                .strafeRight(30)
                .build();
        TrajectorySequence trajSeq2Pos2 = drive.trajectorySequenceBuilder(trajSeq1Pos2.end())
                .lineToLinearHeading(new Pose2d(-53,-15, Math.toRadians(-90)))
                .waitSeconds(.2)
                .back(13)
                .build();
        TrajectorySequence trajSeq3Pos2 = drive.trajectorySequenceBuilder(trajSeq2Pos2.end())
                .forward(28)
                .waitSeconds(.1)
                .strafeLeft(15)
                .build();
        TrajectorySequence trajSeq1Pos1 = drive.trajectorySequenceBuilder(startPose)
                .back(10)
                .waitSeconds(.1)
                .strafeRight(30)
                .build();
        TrajectorySequence trajSeq2Pos1 = drive.trajectorySequenceBuilder(trajSeq1Pos1.end())
                .lineToLinearHeading(new Pose2d(-53,-33, Math.toRadians(-90)))
                .waitSeconds(.2)
                .back(24.5)
                .build();
        TrajectorySequence trajSeq3Pos1 = drive.trajectorySequenceBuilder(trajSeq2Pos1.end())
                .forward(28)
                .waitSeconds(.1)
                .strafeLeft(15)
                .build();

        int finalSlidePos = 0;
        int slidePos = 0;
        ArrayList<Integer> SlideLevelList = new ArrayList<>();
        int loop_count = 5;
        long loop_delay = 1; // delay for tensorflow loop iterations

        telemetry.addData("Hello User", "Bleeding Steel, Online.");
        telemetry.addData(">", "Waiting 5s to initialize tensorflow");
        telemetry.update();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.1, 16.0 / 9.0);
        }



       waitForStart();



        telemetry.addData("Hello User", "Bleeding Steel, Online.");
        telemetry.addData(">", "Waiting 5s to initialize tensorflow");
        telemetry.update();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.1, 16.0 / 9.0);
        }


        for (int j = 0; j < loop_count; j++) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.

                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                if (updatedRecognitions != null) {

                    telemetry.addData("# Object Detected", updatedRecognitions.size());


                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        if (recognition.getLabel().equals("TSE")) {
                            int TSEpos = (int) recognition.getRight();
                            if (TSEpos <= 900 && TSEpos >= 600) {
                                slidePos = 3;
                            } else if (TSEpos <= 550 && TSEpos >= 350) {
                                slidePos = 2;
                            } else if (TSEpos <= 300 && TSEpos >= 0)
                                slidePos = 1;
                        } else {
                            slidePos = 3;
                        }
                    }

                    telemetry.addData("slidePos is", slidePos);
                    //telemetry.update();

                    i++;
                }
                /*else {
                    telemetry.addData(">", "no object detected");
                    telemetry.update();
                }*/
                //Insert into the list
                Log.d("FTC_20288", "count:" + j);
                SlideLevelList.add(slidePos);
                Log.d("FTC_20288", "added to list");
            }
            sleep(loop_delay); // Loop Delay between each tensorflow detection
        } // end of multiple detection for loop

        Log.d("FTC_20288", "List: " + SlideLevelList);

        // Count how many times each level is detected
        /*Set<Integer> set = new HashSet<Integer>(SlideLevelList);
        for (Integer r : set) {
            Log.d("FTC_20288", (r + ": " + Collections.frequency(SlideLevelList, r)));
            int LevelCount = Collections.frequency(SlideLevelList, r);

        }
        int Level1Count = Collections.frequency(SlideLevelList, 1);
        int Level2Count = Collections.frequency(SlideLevelList, 2);
        int Level3Count = Collections.frequency(SlideLevelList, 3);
        Log.d("FTC_20288", ("Count Level1: " + Level1Count + " Count Level2: " + Level2Count + " Count Level3: " + Level3Count));

        // If Levelcount > 5 this is the final slide position
        if (Level1Count > 5) {
            finalSlidePos = 1;
        }
        else if (Level2Count > 5) {
            finalSlidePos = 2;
        }
        else if (Level3Count > 5) {
            finalSlidePos = 3;
        }
        else {
            finalSlidePos = 3; // Set this as default if detection fails
        }*/

        int x = SlideLevelList.get(loop_count - 3); // Third last element of SlideLevelList
        int y = SlideLevelList.get(loop_count - 2); // Second last element of SlideLevelList
        int z = SlideLevelList.get(loop_count - 1); // Last element of SlideLevelList

        //for(int i = SlideLevelList.size()-1; i >= SlideLevelList.size()-3; i--) {
        //Check if the last three elements are equal
        if ((x == y) && (y == z)) {
            finalSlidePos = z;
            Log.d("FTC_20288", "Final 3 positions are: " + x + " | " + y + " | " + z);
        } else {
            finalSlidePos = 3; // fail-safe (assign Level 3 if detection fails)
        }
        //}

        Log.d("FTC_20288", "Final Slide Position: " + finalSlidePos);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();




        Slide.setTargetPosition(-200);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION); BucketReady();
        Vcap.setPosition(.35);

        telemetry.addData("Opmode started - Final slidePos is", slidePos);
        //telemetry.update();

        //set slide heights using encoders for each position (1, 2, 3)
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slide.setPower(0.35);
        if (finalSlidePos == 1) {

            drive.followTrajectorySequence(trajSeq1Pos1);
            Caro.setPower(.34) ;
            sleep(5000);
            Caro.setPower(0);
            drive.followTrajectorySequence(trajSeq2Pos1);
            slidepos1();
            drive.followTrajectorySequence(trajSeq3Pos1);
            BucketReady();
            Vcap.setPosition(.1);




        }
        else if (finalSlidePos == 2) {


            drive.followTrajectorySequence(trajSeq1Pos2);
            Caro.setPower(.34) ;
            sleep(5000);
            Caro.setPower(0);
            drive.followTrajectorySequence(trajSeq2Pos2);
            slidepos2();
            drive.followTrajectorySequence(trajSeq3Pos2);
            BucketReady();
            Vcap.setPosition(.1);

        }
        else if (finalSlidePos == 3) {

            BucketReady();
            drive.followTrajectorySequence(trajSeq1Pos3);
            Caro.setPower(.34) ;
            sleep(5000);
            Caro.setPower(0);
            drive.followTrajectorySequence(trajSeq2Pos3);
           slidepos3();
            drive.followTrajectorySequence(trajSeq3Pos3);
            BucketReady();
            Vcap.setPosition(.1);




        }
        telemetry.update();
        sleep(3000);
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 1080;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

    }
    private void slidepos3(){
        Slide.setTargetPosition(-1655);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        bucket.setPosition(0.4);
        telemetry.addData("Slide Level 3", Slide.getCurrentPosition());
        sleep(1300);
        bucket.setPosition(0.05);
        sleep(1300);
        bucket.setPosition (0.05); //increments of .05 exist
        sleep(1300);
        bucket.setPosition(.85);
        sleep(100);
        Slide.setTargetPosition(-200);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
    }
    private void slidepos2(){
        Slide.setTargetPosition(-1124);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        bucket.setPosition(0.4);
        telemetry.addData("Slide Level 2", Slide.getCurrentPosition());
        sleep(1300);
        bucket.setPosition(0.05);
        sleep(1300);
        bucket.setPosition (0.05); //increments of .05 exist
        sleep(1300);
        bucket.setPosition(.85);
        sleep(100);
        Slide.setTargetPosition(-200);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
    }
    private void slidepos1(){
        Slide.setTargetPosition(-754);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);
        bucket.setPosition(0.4);
        telemetry.addData("Slide Level 1", Slide.getCurrentPosition());
        sleep(1300);
        bucket.setPosition(0.05);
        sleep(1300);
        bucket.setPosition (0.05); //increments of .05 exist
        sleep(1300);
        bucket.setPosition(.85);
        sleep(100);
        Slide.setTargetPosition(-200);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

    }
    private void BucketReady(){
        bucket.setPosition(0.75);
        Slide.setTargetPosition(-100);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}

