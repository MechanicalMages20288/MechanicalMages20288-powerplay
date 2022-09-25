package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.opmode.StraightTest.DISTANCE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;




@Autonomous(group = "Cool Commands")
public class CoolCommands extends LinearOpMode {

    DcMotor Caro;
    DcMotor Slide;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Caro = hardwareMap.dcMotor.get("Caro");
        Slide = hardwareMap.dcMotor.get("Slide");

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, -26, Math.toRadians(-90)))
                .build();
        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(30, 40), Math.toRadians(0))
                .build();
        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(11)
                .build();



        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq1);
        drive.followTrajectorySequence(trajSeq2);
        drive.followTrajectorySequence(trajSeq3);
    }
}