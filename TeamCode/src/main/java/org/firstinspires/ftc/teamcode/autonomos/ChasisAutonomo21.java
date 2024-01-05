package org.firstinspires.ftc.teamcode.autonomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

//@Disabled
@Autonomous(name="ChasisAutonomo2.1", group="Pushbot")

public class ChasisAutonomo21 extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot = new RobotConfigMaster_RR();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, -62, Math.toRadians(90));
        robot.init(hardwareMap, telemetry);
        drive.setPoseEstimate(startPose);

        Pose2d nadia = drive.getPoseEstimate();
        Pose2d uwu = new Pose2d(nadia.getX(), nadia.getY(), nadia.getHeading());


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-42.5, -45))
                .build();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq.end())
                .forward(9.5)
                .strafeRight(11.5)
                .build();


        //Izquierda Detectada
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(-46, -15, Math.toRadians(270  - 1e-6)))
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .back(15)
                .lineToLinearHeading(new Pose2d(38,-14, Math.toRadians(0)))
                .build();

        //Centro Detectado
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .lineToSplineHeading(new Pose2d(-47, -22, Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq4.end())
                .back(5)
                .strafeLeft(10)
                .forward(74)
                .build();

        //Derecho
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .strafeLeft(3.5)
                        .turn(Math.toRadians(-90))
                .strafeLeft(2)
                .build();

        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq6.end())
                .back(10)
                .strafeLeft(25)
                .forward(65)
                .build();

        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            if (robot.distanciaCentimetros()<40){
                drive.followTrajectorySequence(trajSeq1);
                drive.followTrajectorySequence(trajSeq2);
            } else {
                drive.followTrajectorySequence(trajSeq3);
                if (robot.distanciaCentimetros()<40){
                    drive.followTrajectorySequence(trajSeq4);
                    drive.followTrajectorySequence(trajSeq5);
                } else
                    drive.followTrajectorySequence(trajSeq6);
                drive.followTrajectorySequence(trajSeq7);
            }
        }


    }
}
