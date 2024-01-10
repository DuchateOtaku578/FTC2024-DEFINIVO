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
        sleep(500);
        robot.subirGarra();

        Pose2d nadia = drive.getPoseEstimate();
        Pose2d uwu = new Pose2d(nadia.getX(),nadia.getY(),nadia.getHeading());



        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .strafeRight(5)
                .build();

        TrajectorySequence trajSec2 = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(-38, 14.5 , Math.toRadians(90)))
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
                .back(10).addDisplacementMarker(0.1, () -> {
                    robot.subirGarra();
                })
                .lineToSplineHeading(new Pose2d(30, 5, Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .lineToLinearHeading(new Pose2d(-45,15, Math.toRadians(-270)))
                .build();

        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .back(15)
                .lineToLinearHeading(new Pose2d(30, 5,Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq9 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .back(10)
                .strafeLeft(22)
                .forward(69)
                .build();




        TrajectorySequence trajSeq12 = drive.trajectorySequenceBuilder(trajSec2.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(-59.5,12.5,Math.toRadians(180))).addSpatialMarker(new Vector2d(-87,12),()->{
                    robot.abrirGarraIzq();
                })
                .build();

        TrajectorySequence trajSeq10 = drive.trajectorySequenceBuilder(trajSeq6.end())
                .strafeTo(new Vector2d(49,37)).addTemporalMarker(0.1, ()->{
                    robot.subirElevador(0.5);
                })
                .build();

        TrajectorySequence trajseq11 = drive.trajectorySequenceBuilder(trajSeq10.end())
                .back(10)
                .strafeLeft(20).addTemporalMarker(0.03,()->{
                    robot.bajarElevador(0.3);
                })
                .forward(15)
                .build();




        telemetry.update();

        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            if(robot.distanciaCentimetros() <=30){
                //morado
                drive.followTrajectorySequence(trajSec2);
                robot.bajarGarra();
                robot.abrirGarraIzq();
                sleep(100);
                robot.cerrarGarraIzq();
                robot.subirGarra();
                //amarillo
                drive.followTrajectorySequence(trajSeq6);
                drive.followTrajectorySequence(trajSeq10);
                robot.abrirGarraDer();
                sleep(100);
                drive.followTrajectorySequence(trajseq11);


            }else {
                drive.followTrajectorySequence(trajSeq3);
                if (robot.distanciaCentimetros() <=30) {
                    drive.followTrajectorySequence(trajSeq7);
                    robot.bajarGarra();
                    robot.abrirGarraIzq();
                    sleep(100);
                    robot.subirGarra();
                    drive.followTrajectorySequence(trajSeq8);
                    drive.followTrajectorySequence(trajSeq10);
                    robot.abrirGarraDer();
                    sleep(100);
                    drive.followTrajectorySequence(trajseq11);
                } else {
                    drive.followTrajectorySequence(trajSeq5);
                    drive.followTrajectorySequence(trajSeq9);
                }

            }
        }


    }
}
