package org.firstinspires.ftc.teamcode.autonomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Mat;

import kotlin.time.FormatToDecimalsKt;

//@Disabled
@Autonomous(name="ChasisAutonomo2", group="Pushbot")

public class ChasisAutonomo2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot = new RobotConfigMaster_RR();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, 62, Math.toRadians(270));
        robot.init(hardwareMap, telemetry);
        drive.setPoseEstimate(startPose);

        Pose2d nadia = drive.getPoseEstimate();
        Pose2d uwu = new Pose2d(nadia.getX(),nadia.getY(),nadia.getHeading());



        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(18)
                .strafeRight(5)
                .build();

        TrajectorySequence trajSec2 = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(-38, 14, Math.toRadians(90)))
                .build();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq.end())
                .strafeRight(11)
                .build();

       /* TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .lineToLinearHeading(new Pose2d(-52,0, Math.toRadians(90)))
                .build();
*/
        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .lineToLinearHeading(new Pose2d(-34.5,35.5, Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajSec2.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(30, 5, Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .lineToLinearHeading(new Pose2d(-45,16, Math.toRadians(90)))
                .build();

        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .back(15)
                .lineToLinearHeading(new Pose2d(30, 8,Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq9 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .back(10)
                .strafeLeft(22)
                .forward(69)
                .build();

        telemetry.update();

        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            if(robot.distanciaCentimetros() <40){
                drive.followTrajectorySequence(trajSec2);
                drive.followTrajectorySequence(trajSeq6);
            }else {
                drive.followTrajectorySequence(trajSeq3);
                if (robot.distanciaCentimetros() < 40) {
                   // drive.followTrajectorySequence(trajSeq4);
                    drive.followTrajectorySequence(trajSeq7);
                    drive.followTrajectorySequence(trajSeq8);
                } else {
                    drive.followTrajectorySequence(trajSeq5);
                    drive.followTrajectorySequence(trajSeq9);
                }

            }
        }


    }
}
