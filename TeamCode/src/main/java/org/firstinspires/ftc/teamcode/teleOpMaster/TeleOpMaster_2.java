package org.firstinspires.ftc.teamcode.teleOpMaster;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOpMaster_SOLO", group="Pushbot")

public class TeleOpMaster_2 extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot  = new RobotConfigMaster_RR();
        robot.init(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        final double dispararAvion = 0.25;

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();
            //Telemetry del robot
            telemetry.addLine("------------Variables y estadisticas del robot------------");
            telemetry.addLine("------------Chasis------------");

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Posicion en X: ", poseEstimate.getX());
            telemetry.addData("Posiscion en Y: ", poseEstimate.getY());
            telemetry.addData("Angulo de orientacion: ", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addLine("------------Elevador------------");
            telemetry.addData("Pulsos elevador Derecha: ", robot.elevador_1.getCurrentPosition());
            telemetry.addData("Puslos elevador Izquierda: ", robot.elevador_2.getCurrentPosition());
            telemetry.addLine("--------Sensor-----------");
            telemetry.addData("Distancia al objeto mas cercano CM", robot.distanciaCentimetros());
            telemetry.update();

            //control elevador
            if(gamepad1.right_bumper){
                robot.subirElevador(1);
            } else if(gamepad1.left_bumper && robot.elevador_1.getCurrentPosition() > 0 && robot.elevador_2.getCurrentPosition() > 0){
                robot.bajarElevador(1);
            }else {
                robot.mantenerElevador();
            }

            //controles de la garra

            if(gamepad1.left_trigger >0.1){
                robot.abrirGarraIzq();
            }else
                robot.cerrarGarraIzq();

           if(gamepad1.right_trigger > 0.1){
                robot.abrirGarraDer();
            }else
                robot.cerrarGarraDer();

            if(gamepad1.a){
                robot.bajarGarra();
            }else if(gamepad1.y){
                robot.subirGarra();
            }

          /*  if(gamepad1.dpad_up){
                robot.enrollarGancho();
            }else if(gamepad1.dpad_down) {
                robot.desenrrollarGancho();
            }else {
                robot.mantenerGancho();
            }*/

            if(gamepad1.x && gamepad1.left_bumper){
                robot.ligaAvion.setPosition(0.4);
            }

        }
    }
}



