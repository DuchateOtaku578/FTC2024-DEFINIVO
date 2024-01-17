package org.firstinspires.ftc.teamcode.autonomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.opencv.core.Mat;

//@Disabled
// Gabo = Arnold
//Beto = úrsula
//Nadia = Olga
//Orión = Petra
@Autonomous(name="Autonomo rojo izq M->D", group="Pushbot")

public class ChasisAutonomo21 extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot = new RobotConfigMaster_RR();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, -62, Math.toRadians(90));
        robot.init(hardwareMap, telemetry);
        sleep(500);
        drive.setPoseEstimate(startPose);
        robot.subirGarra();

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
                .lineToLinearHeading(new Pose2d(-47, -15, Math.toRadians(270  - 1e-6))).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .back(13)
                .lineToLinearHeading(new Pose2d(38,-12, Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.subirGarra();
                    robot.cerrarGarraDer();
                })
                .splineToConstantHeading(new Vector2d(51, -28.7), Math.toRadians(0))
                .build();

        //Centro Detectado
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .strafeLeft(10)
                .lineToSplineHeading(new Pose2d(-39, -14.5, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq4.end())
                .back(15).addTemporalMarker(0.1 , () ->{
                    robot.subirGarra();
                    robot.cerrarGarraDer();
                })
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(0)))
                .build();

        //Derecho
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .strafeTo(new Vector2d(-55, -34))
                .lineToLinearHeading(new Pose2d(-35, -33, Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq6.end())
                .back(10).addTemporalMarker(0.1, () ->{
                    robot.subirGarra();
                    robot.cerrarGarraDer();
                })
                .lineToSplineHeading(new Pose2d(-34, 0,Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(0)))
                .build();

        TrajectorySequence trayectoria11 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .back(10)
                .strafeRight(29.5).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.7);
                }).addTemporalMarker(0.1 + 0.5, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(14)
                .build();

        TrajectorySequence trayectoria12 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .splineToConstantHeading(new Vector2d(48, -34),Math.toRadians(0))
                .build();

        TrajectorySequence trayectoria13 = drive.trajectorySequenceBuilder(trayectoria12.end())
                .back(10)
                .strafeRight(22.5).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.7);
                }).addTemporalMarker(0.1 + 0.6, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(14)
                .build();

        TrajectorySequence trayectoria14 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .splineToConstantHeading(new Vector2d(49.5, -41),Math.toRadians(0)).addTemporalMarker(0.1, () ->{
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.1 + 1.2, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence trayectorira15 = drive.trajectorySequenceBuilder(trayectoria14.end())
                .back(10).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.7);
                }).addTemporalMarker(0.1 + 0.6, () ->{
                    robot.mantenerElevadorBrake();
                    robot.cerrarGarraIzq();
                })
                .strafeRight(18)
                .forward(15)
                .build();

        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            if (robot.distanciaCentimetros()<=20){
                drive.followTrajectorySequence(trajSeq1);
                robot.abrirGarraDer();
                sleep(300);
                drive.followTrajectorySequence(trajSeq2);
                robot.subirElevador(0.7);
                sleep(1200);
                robot.mantenerElevadorBrake();
                robot.abrirGarraIzq();
                sleep(200);
                drive.followTrajectorySequence(trayectoria11);
            } else {
                drive.followTrajectorySequence(trajSeq3);
                if( robot.distanciaCentimetros()<=20){
                    drive.followTrajectorySequence(trajSeq4);
                    robot.bajarGarra();
                    robot.abrirGarraDer();
                    sleep(500);
                    drive.followTrajectorySequence(trajSeq5);
                    drive.followTrajectorySequence(trayectoria12);
                    robot.subirElevador(0.7);
                    sleep(1100);
                    robot.mantenerElevadorBrake();
                    robot.abrirGarraIzq();
                    sleep(500);
                    drive.followTrajectorySequence(trayectoria13);

                } else {
                    drive.followTrajectorySequence(trajSeq6);
                    robot.abrirGarraDer();
                    sleep(400);
                    drive.followTrajectorySequence(trajSeq7);
                    drive.followTrajectorySequence(trayectoria14);
                    robot.abrirGarraIzq();
                    sleep(500);
                    drive.followTrajectorySequence(trayectorira15);


                }

            }
        }


    }
}
