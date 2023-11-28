package org.firstinspires.ftc.teamcode.test.odometria;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.domain.Chasis;

@TeleOp(name="ChasisOdometria",group = "PushBot")

public class ChasisOdometria extends LinearOpMode {

    @Override
    public void runOpMode(){
        ChasisOdometriaConfig robot = new ChasisOdometriaConfig();
        robot.init(hardwareMap, telemetry);
        Chasis chasis = new Chasis(robot.enfrenteDer, robot.enfrenteIzq, robot.atrasDer, robot.atrasIzq, robot.imu, this);

        telemetry.addLine("Odometria inicializada");
        telemetry.update();
        waitForStart();

        double currentRightPos = 0;
        double currentLeftPos = 0;
        double currentAuxPos = 0;

        double oldRightPos = 0;
        double oldLeftPos = 0;
        double oldAuxPos = 0;

        double lastXPos = 0;
        double lastYPos = 0;
        double lastThetaPos = 0;

        double deltaTickDer = 0;
        double deltaTickIzq = 0;
        double deltaTickAux = 0;

        double currentAngle;

        final double encoderTicksPerRevolution = 8192; //ticks por revolucion del encoder

        final double wheelRadius = 2.31; //radius wheel in centimeretrs

        final double large =24; //distancia entre los dos encoders paralelos (pendiente)

        final double middlePoint = 12; //distancia entre el punto central de los 2 encoders y el encoder Delta (pendiente)

        final double cm_per_tick = 2.0 * Math.PI * wheelRadius/encoderTicksPerRevolution; //distancia que recorre un encoder por tick en centimetro

        robot.enfrenteDer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.enfrenteIzq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.atrasDer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        while(opModeIsActive()) {

            currentAngle = robot.obtenerAngulo();
            oldRightPos = currentRightPos;
            oldLeftPos = currentLeftPos;
            oldAuxPos = currentAuxPos;

            currentRightPos = robot.encoderDer.getCurrentPosition();
            currentLeftPos = robot.encoderDer.getCurrentPosition();
            currentAuxPos = robot.encoderTheta.getCurrentPosition();

            deltaTickDer = currentRightPos - oldRightPos;
            deltaTickIzq = currentLeftPos - oldLeftPos;
            deltaTickAux = currentAuxPos - oldAuxPos;

            double rightDistance = ((2 * Math.PI * wheelRadius) * (deltaTickDer) / encoderTicksPerRevolution);
            double leftDistance = ((2 * Math.PI * wheelRadius) * (deltaTickIzq) / encoderTicksPerRevolution);
            double auxDistance = ((2 * Math.PI * wheelRadius) * (deltaTickAux) / encoderTicksPerRevolution);

            double centralDistance = (rightDistance + leftDistance) / 2;

            double currentThetaPos = lastThetaPos + (rightDistance - leftDistance) / large;
            double currentXPos = lastXPos + centralDistance * Math.cos(currentAngle);
            double currentYPos = lastYPos + centralDistance * Math.sin(currentAngle);

            double currentThehtaPosDegrees = Math.toDegrees(currentThetaPos);
            lastXPos = currentXPos;
            lastYPos = currentYPos;
            lastThetaPos = currentThetaPos;

            telemetry.addData("Posicion X: ", currentXPos);
            telemetry.addData("Posicion Y: ", currentYPos);
            telemetry.addData("Posicion Theta: " , currentThehtaPosDegrees);
            telemetry.addData("Angulo de orientacion: ", currentAngle);
            telemetry.update();

            double incremento = (gamepad1.right_stick_button || gamepad1.left_stick_button) ? 1 : 0;

            if(gamepad1.left_stick_y > 0.5){

                chasis.moverseAtras(0.5 + incremento);

            }else if(gamepad1.left_stick_y < -0.5){

                chasis.moverseEnfrente(0.5 + incremento);

            }else if(gamepad1.left_stick_x > 0.5){

                 chasis.moverseDerecha(0.5 + incremento);

            }else if(gamepad1.left_stick_x < -0.5){

                chasis.moverseIzquierda(0.5 + incremento);

            }else if(gamepad1.right_stick_x > 0.5){

                chasis.girarDerecha(0.5 + incremento);

            }else if(gamepad1.right_stick_x < -0.5){

                chasis.girarIzquierda(0.5  + incremento);
            }else
                chasis.parar();

        }
    }
}

