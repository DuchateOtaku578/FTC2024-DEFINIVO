package org.firstinspires.ftc.teamcode.autonomos;

import android.media.tv.TvTrackInfo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster;
import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(name="Autonomo azul izq M->D", group="Pushbot")

public class ChasisAutonomo22 extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot = new RobotConfigMaster_RR();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(270));
        robot.init(hardwareMap, telemetry);
        drive.setPoseEstimate(startPose);
        sleep(500);

        robot.subirGarra();

        Pose2d nadia = drive.getPoseEstimate();

        TrajectorySequence trayectoria1 = drive.trajectorySequenceBuilder(startPose)
                .forward(17)
                .strafeLeft(6)
                .build();

        TrajectorySequence trayectoria2 = drive.trajectorySequenceBuilder(trayectoria1.end())
                .forward(12)
                .build();

        TrajectorySequence trayectoria3 = drive.trajectorySequenceBuilder(trayectoria2.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(52.5, 34 , Math.toRadians(0))).addTemporalMarker(0.1, () -> {
                    robot.subirElevador(1);
                }).addTemporalMarker(0.1 + 0.9, () -> {
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence trayectoria4 = drive.trajectorySequenceBuilder(trayectoria3.end())
                .back(10).addTemporalMarker(0.1, () -> {
                    robot.bajarElevador(1);
                }).addTemporalMarker(0.1 + 0.7, () -> {
                    robot.mantenerElevadorBrake();
                })
                .strafeLeft(25)
                .forward(15)

                .build();

        TrajectorySequence trayectoria5 = drive.trajectorySequenceBuilder(trayectoria1.end())
                .strafeLeft(11)
                .build();

        TrajectorySequence trayectoria6 = drive.trajectorySequenceBuilder(trayectoria5.end())
                .strafeRight(4)
                .build();

        TrajectorySequence trayectoria7 = drive.trajectorySequenceBuilder(trayectoria6.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(53, 40 ,Math.toRadians(0))).addTemporalMarker(
                        0.5, () ->{
                            robot.subirGarra();
                            robot.subirElevador(0.8);
                        }
                ).addTemporalMarker(0.5 + 0.9 , () ->{
                    robot.mantenerElevadorBrake();
                })

                .build();

        TrajectorySequence trayectoria8 = drive.trajectorySequenceBuilder(trayectoria7.end())
                .back(10)
                .strafeLeft(20).addTemporalMarker(0.1 ,() ->{
                   robot.bajarElevador(0.8);
             }).addTemporalMarker(0.1 + 0.7, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(15)
                .build();

        TrajectorySequence trayectoria9 = drive.trajectorySequenceBuilder(trayectoria5.end())
                .lineToLinearHeading(new Pose2d(11,33 , Math.toRadians(180))).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence trayectoria10 = drive.trajectorySequenceBuilder(trayectoria9.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(53.5,27, Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.subirGarra();
                    robot.subirElevador(0.8);
                }).addTemporalMarker(0.1 + 1, () ->{
                    robot.mantenerElevadorBrake();
        })
                .build();

        TrajectorySequence trayectoria11 = drive.trajectorySequenceBuilder(trayectoria10.end())
                .back(10)
                .strafeLeft(32).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.8);
                }).addTemporalMarker(0.1+ 0.8, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(10)
                .build();




        telemetry.update();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trayectoria1);
            if (robot.distanciaCentimetros_2() <= 40) {
                    robot.bajarGarra();
                    drive.followTrajectorySequence(trayectoria2);
                    robot.abrirGarraDer();
                    sleep(500);
                    robot.subirGarra();
                    drive.followTrajectorySequence(trayectoria3);
                    robot.abrirGarraIzq();
                    drive.followTrajectorySequence(trayectoria4);


            } else {
                    drive.followTrajectorySequence(trayectoria5);
                if (robot.distanciaCentimetros_2() <= 20) {
                    drive.followTrajectorySequence(trayectoria6);
                    robot.bajarGarra();
                    robot.abrirGarraDer();
                    sleep(300);
                    robot.subirGarra();
                    drive.followTrajectorySequence(trayectoria7);
                    robot.abrirGarraIzq();
                    sleep(300);
                    drive.followTrajectorySequence(trayectoria8);

                } else {
                    drive.followTrajectorySequence(trayectoria9);
                    robot.abrirGarraDer();
                    drive.followTrajectorySequence(trayectoria10);
                    robot.abrirGarraIzq();
                    drive.followTrajectorySequence(trayectoria11);




                }
            }


        }
    }
}
