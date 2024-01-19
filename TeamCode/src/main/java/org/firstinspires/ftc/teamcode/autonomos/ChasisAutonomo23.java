package org.firstinspires.ftc.teamcode.autonomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(name="Autonomo rojo der M->D", group="Pushbot")

public class ChasisAutonomo23 extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot = new RobotConfigMaster_RR();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(90));
        robot.init(hardwareMap, telemetry);
        drive.setPoseEstimate(startPose);
        sleep(500);
        robot.subirGarra();


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .strafeRight(5)
                .build();

        TrajectorySequence trayectoria2 = drive.trajectorySequenceBuilder(trajSeq.end())
                .strafeRight(12.5)
                .build();

        TrajectorySequence trayectoria3 = drive.trajectorySequenceBuilder(trajSeq.end())
                .forward(9).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .strafeLeft(3)
                .build();

        TrajectorySequence trayectoria4 = drive.trajectorySequenceBuilder(trayectoria3.end())
                .back(5)
                .lineToSplineHeading(new Pose2d(52, -37.2, Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.1 + 1.2, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();
//0.9-0.7
        TrajectorySequence trayectoria5 = drive.trajectorySequenceBuilder(trayectoria4.end())
                .back(10).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.8);
                }).addTemporalMarker(0.1 + 0.7, () -> {
                    robot.mantenerElevadorBrake();
                })
                .strafeRight(25)
                .forward(15)
                .build();

        TrajectorySequence trayectoria6 = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineTo(new Vector2d(22, -34))
                .build();

        TrajectorySequence trayectoria7 = drive.trajectorySequenceBuilder(trayectoria6.end())
                .back(10).addTemporalMarker(0.1, () ->{
                    robot.subirGarra();
                    robot.subirElevador(0.8);
                }).addTemporalMarker(0.1 + 1, () -> {
                    robot.mantenerElevadorBrake();
                }).lineToSplineHeading(new Pose2d(52, -43, Math.toRadians(0)))
                        .build();

        TrajectorySequence trayectoria8 = drive.trajectorySequenceBuilder(trayectoria7.end())
                .back(10).addTemporalMarker(0.1, () -> {
                    robot.subirGarra();
                    robot.cerrarGarraIzq();
                    robot.cerrarGarraDer();
                    robot.bajarElevador(0.8);
                }).addTemporalMarker(0.1 + 0.8, () -> {
                    robot.mantenerElevadorBrake();
                })
                .strafeRight(20)
                .forward(16.5)
                        .build();

        TrajectorySequence trajectoria9 = drive.trajectorySequenceBuilder(trayectoria2.end())
                .lineToSplineHeading(new Pose2d(10.5,-33, Math.toRadians(180))).addTemporalMarker(0.1, () -> {
                    robot.bajarGarra();
                })
                        .build();

    TrajectorySequence trajectoria10 = drive.trajectorySequenceBuilder(trajectoria9.end())
            .back(10)
            .lineToSplineHeading(new Pose2d(52 , -29
                    , Math.toRadians(0))).addTemporalMarker(0.1,() ->{
                robot.subirGarra();
                robot.subirElevador(0.7);
        }).addTemporalMarker(0.1 + 1.1 , () ->{
            robot.mantenerElevadorBrake();
            })
                    .build();

    TrajectorySequence trayectoria11 = drive.trajectorySequenceBuilder(trajectoria10.end())
            .back(10)
            .strafeRight(28).addTemporalMarker(0.1, () ->{
                robot.bajarElevador(0.7);
            }).addTemporalMarker(0.1 + 0.7, () ->{
                robot.mantenerElevadorBrake();
            })
            .forward(14)
            .build();

        telemetry.update();

        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            if(robot.distanciaCentimetros() <=30){
                drive.followTrajectorySequence(trayectoria3);
                robot.abrirGarraDer();
                sleep(200);
                robot.subirGarra();
                drive.followTrajectorySequence(trayectoria4);
                robot.abrirGarraIzq();
                sleep(400);
                drive.followTrajectorySequence(trayectoria5);



            }else {
                drive.followTrajectorySequence(trayectoria2);
                if (robot.distanciaCentimetros() <=30) {
                    robot.bajarGarra();
                    sleep(300);
                    drive.followTrajectorySequence(trayectoria6);
                    robot.abrirGarraDer();
                    sleep(200);
                    drive.followTrajectorySequence(trayectoria7);
                    robot.abrirGarraIzq();
                    sleep(300);
                    drive.followTrajectorySequence(trayectoria8);


                } else {
                    drive.followTrajectorySequence(trajectoria9);
                    robot.abrirGarraDer();
                    sleep(200);
                    drive.followTrajectorySequence(trajectoria10);
                    robot.abrirGarraIzq();
                    sleep(200);
                    drive.followTrajectorySequence(trayectoria11);




                }

            }
        }


    }
}
