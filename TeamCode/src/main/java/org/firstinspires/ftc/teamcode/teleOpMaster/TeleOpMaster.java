package org.firstinspires.ftc.teamcode.teleOpMaster;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="TeleOpMaster", group="Pushbot")

public class TeleOpMaster extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot  = new RobotConfigMaster_RR();
        robot.init(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            //Telemetry del robot
            telemetry.addLine("------------Variables y estadisticas del robot------------");
            telemetry.addLine("------------Chasis------------");
            telemetry.addLine("-----Motor enfrenteDer----- ");
            telemetry.addData("Potencia: ", drive.rightFront.getPower());
            telemetry.addData("Velocidad: ", drive.rightFront.getVelocity());
            telemetry.addLine("-----Motor enfrenteIzq----- ");
            telemetry.addData("Potencia: ", drive.leftFront.getPower());
            telemetry.addData("Velocidad: ", drive.leftFront.getVelocity());
            telemetry.addLine("-----Motor atrasDer----- ");
            telemetry.addData("Potencia: ", drive.rightRear.getPower());
            telemetry.addData("Velocidad: ", drive.rightRear.getVelocity());
            telemetry.addLine("-----Motor atrasIzq-----");
            telemetry.addData("Potencia: ", drive.leftRear.getPower());
            telemetry.addData("Velocidad: ", drive.leftRear.getVelocity());
            //Posicion del Chasis------------------------------------------
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Posicion en X: ", poseEstimate.getX());
            telemetry.addData("Posiscion en Y: ", poseEstimate.getY());
            telemetry.addData("Angulo de orientacion: ", poseEstimate.getHeading());
            
            telemetry.update();
            //-------------------------------------------------------------
            telemetry.addLine("------------Elevador------------");
            telemetry.addData("Pulsos elevador Derecha: ", robot.elevador_1.getCurrentPosition());
            telemetry.addData("Puslos elevador Izquierda: ", robot.elevador_2.getCurrentPosition());


            //control elevador
            if(gamepad1.right_bumper){
                robot.subirElevador();
                telemetry.addData("Pulsos elevador Derecha: ", robot.elevador_1.getCurrentPosition());
                telemetry.addData("Puslos elevador Izquierda: ", robot.elevador_2.getCurrentPosition());
            } else if(gamepad1.left_bumper && robot.elevador_1.getCurrentPosition() > 0 && robot.elevador_2.getCurrentPosition() > 0){
                robot.bajarElevador();
                telemetry.addData("Pulsos elevador Derecha: ", robot.elevador_1.getCurrentPosition());
                telemetry.addData("Puslos elevador Izquierda: ", robot.elevador_2.getCurrentPosition());
            }else {
                robot.mantenerElevador();
                telemetry.addData("Pulsos elevador Derecha: ", robot.elevador_1.getCurrentPosition());
                telemetry.addData("Puslos elevador Izquierda: ", robot.elevador_2.getCurrentPosition());
            }

            telemetry.update();
            //controles de la garra

           if(gamepad1.right_trigger > 0.1){
                robot.abrirGarra();
            }else
                robot.cerrarGarra();

            if(gamepad1.a){
                robot.bajarGarra();
            }else if(gamepad1.y){
                robot.subirGarra();
            }

            if(gamepad1.dpad_up){
                robot.enrollarGancho();
            }else if(gamepad1.dpad_down) {
                robot.desenrrollarGancho();
            }else
                robot.mantenerGancho();


        }
    }
}



