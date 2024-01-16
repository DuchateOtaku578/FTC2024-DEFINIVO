package org.firstinspires.ftc.teamcode.autonomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(name="ChasisAutonomo2.3", group="Pushbot")

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
                .forward(8).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .strafeLeft(3)
                .build();

        TrajectorySequence trayectoria4 = drive.trajectorySequenceBuilder(trayectoria3.end())
                .back(5)
                .lineToSplineHeading(new Pose2d(53, -35, Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.1 + 0.8, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence trayectoria5 = drive.trajectorySequenceBuilder(trayectoria4.end())
                .back(10).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.7);
                }).addTemporalMarker(0.1 + 0.6, () -> {
                    robot.mantenerElevadorBrake();
                })
                .strafeRight(25)
                .forward(15)
                .build();


        TrajectorySequence trayectoria6 = drive.trajectorySequenceBuilder(trayectoria2.end())
                .back(10)
                .lineToLinearHeading(new Pose2d(53, -38,Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.1 + 0.8, () ->{
                    robot.mantenerElevadorBrake();
                })

                .build();

        TrajectorySequence trayectoria7 = drive.trajectorySequenceBuilder(trayectoria2.end())
                .back(3).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .strafeLeft(5)
                .build();

        TrajectorySequence trayectoria8 = drive.trajectorySequenceBuilder(trayectoria7.end())
                .back(10).addTemporalMarker(0.1 + 0.8, () ->{
                    robot.bajarElevador(0.6);
                    robot.subirGarra();
                }).addTemporalMarker(0.1 + 0.2, () ->{
                    robot.mantenerElevadorBrake();
                })
                .strafeRight(19)
                .forward(10)
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
                drive.followTrajectorySequence(trayectoria5);



            }else {
                drive.followTrajectorySequence(trayectoria2);
                if (robot.distanciaCentimetros() <=30) {
                    drive.followTrajectorySequence(trayectoria7);
                    robot.abrirGarraDer();
                    sleep(200);
                    robot.subirGarra();
                    drive.followTrajectorySequence(trayectoria6);
                    robot.abrirGarraIzq();
                    sleep(200);
                    drive.followTrajectorySequence(trayectoria8);

                } else {




                }

            }
        }


    }
}
