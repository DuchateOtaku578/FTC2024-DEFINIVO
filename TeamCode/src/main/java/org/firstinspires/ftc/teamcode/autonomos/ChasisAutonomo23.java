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
                .lineToLinearHeading(new Pose2d(-34.5,33, Math.toRadians(0))).addTemporalMarker(0.2, () ->{
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajSec2.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(30, 5, Math.toRadians(0)))
                .build();

        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .back(2)
                .strafeLeft(3)
                .build();

        TrajectorySequence trajSeq8 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .back(15)
                .strafeLeft(13)
                .forward(48)
                .lineToSplineHeading(new Pose2d(30, 5,Math.toRadians(0))).addTemporalMarker(0.5, () ->{
                    robot.subirGarra();
                    robot.cerrarGarraIzq();
                })
                .build();

        TrajectorySequence trajSeq9 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .back(10).addTemporalMarker(0.5, () ->{
                    robot.cerrarGarraIzq();
                    robot.subirGarra();
                })
                .strafeTo(new Vector2d(-35, 13))
                .splineToConstantHeading(new Vector2d(20, 5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(52, 45), Math.toRadians(0)).addTemporalMarker(
                        2, () ->{
                            //robot.subirElevador(0.7);
                        }
                ).addTemporalMarker(2 + 0.8, () ->{
                  // robot.mantenerElevadorBrake();
                })

                .build();




        TrajectorySequence trajSeq12 = drive.trajectorySequenceBuilder(trajSec2.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(-59.5,12.5,Math.toRadians(180))).addSpatialMarker(new Vector2d(-87,12),()->{
                    robot.abrirGarraIzq();
                })
                .build();

        TrajectorySequence trajSeq10 = drive.trajectorySequenceBuilder(trajSeq6.end())
                .strafeTo(new Vector2d(54,37)).addTemporalMarker(0.2, ()->{
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.2 + 0.8,() ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence trajseq11 = drive.trajectorySequenceBuilder(trajSeq10.end())
                .back(10)
                .strafeLeft(22).addTemporalMarker(0.2,()->{
                    robot.bajarElevador(0.7);
                    robot.subirGarra();
                }).addTemporalMarker(0.2 + 1, () -> {
                   robot.mantenerElevadorBrake();
               })
                .forward(15)
                .build();

        TrajectorySequence trajSeq13 = drive.trajectorySequenceBuilder(trajSeq8.end())
                .strafeTo(new Vector2d(53,30)).addTemporalMarker(0.2, ()->{
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.2 + 0.8,() ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence trajseq14 = drive.trajectorySequenceBuilder(trajSeq13.end())
                .back(10)
                .strafeLeft(27).addTemporalMarker(0.2,()->{
                    robot.bajarElevador(0.7);
                    robot.subirGarra();
                }).addTemporalMarker(0.2 +0.8, () -> {
                    robot.mantenerElevadorBrake();
                })
                .forward(15)
                .build();

        TrajectorySequence trajeseq15 = drive.trajectorySequenceBuilder(trajSeq9.end())
                .back(10)
                .strafeLeft(18)
                .forward(13)
                .build();



        telemetry.update();

        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            if(robot.distanciaCentimetros() <=20){
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
                if (robot.distanciaCentimetros() <=20) {
                    drive.followTrajectorySequence(trajSeq7);
                    robot.bajarGarra();
                    robot.abrirGarraIzq();
                    sleep(400);
                    robot.subirGarra();
                    drive.followTrajectorySequence(trajSeq8);
                    drive.followTrajectorySequence(trajSeq13);
                    robot.abrirGarraDer();
                    sleep(500);
                    drive.followTrajectorySequence(trajseq14);
                } else {
                    drive.followTrajectorySequence(trajSeq5);
                    robot.bajarGarra();
                    robot.abrirGarraIzq();
                    sleep(400);
                    robot.subirGarra();
                    drive.followTrajectorySequence(trajSeq9);
                    robot.subirElevador(0.7);
                    sleep(1500);
                    robot.mantenerElevadorBrake();
                    robot.abrirGarraDer();
                    drive.followTrajectorySequence(trajeseq15);
                    robot.bajarElevador(0.7);
                    sleep(1000);



                }

            }
        }


    }
}
