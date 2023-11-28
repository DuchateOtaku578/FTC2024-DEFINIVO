package org.firstinspires.ftc.teamcode.test.elevador;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="PreubaElevador", group="Pushbot")
//@Disabled
public class PreubaElevador extends LinearOpMode {
    PruebaMotoresPosicionConfig_2 robot = new PruebaMotoresPosicionConfig_2();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        resetearEncoder();
        while (opModeIsActive()) {
            double stickY = -gamepad2.left_stick_y;
            telemetry.update();

            if (gamepad1.b) {
                robot.motor.setPower(1);
                robot.motor_2.setPower(1);
            } else if (gamepad1.x) {
                robot.motor.setPower(-1);
                robot.motor_2.setPower(-1);
            } else {
                pararMotores();
            }
        }


    }

        public void usingEncoder () {
            robot.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void resetearEncoder () {
            robot.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        public void setTarget ( int distancia){
            robot.motor.setTargetPosition(distancia);
        }

        public void setRunToPosition () {
            robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void activarMotor ( int potencia){
            robot.motor.setPower(potencia);
        }

        public void mantenerse () {

            robot.motor.setTargetPosition(robot.motor.getCurrentPosition());

            robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motor.setPower(1);

            robot.motor_2.setTargetPosition(robot.motor_2.getCurrentPosition());

            robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.motor_2.setPower(1);

        }

        public void moverseDistanciaMantener ( double potencia, int distance){
            robot.motor.setTargetPosition(distance);
            telemetry.addLine("Set target position");
            telemetry.update();

            robot.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addLine("Run to position");
            telemetry.update();

            moverseEnfrente(potencia);
            telemetry.addLine("Moverse Enfrente");
            telemetry.update();

            while (robot.motor.isBusy()) {
                telemetry.addLine("Dentro del Busy");
                telemetry.update();
            }

        }

        public void moverseEnfrente ( double potencia){
            robot.motor.setPower(potencia);
            robot.motor.setPower(potencia);
        }

        public void pararMotores () {
            robot.motor.setPower(0);
            robot.motor_2.setPower(0);
        }

    }

