package org.firstinspires.ftc.teamcode.test.servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PruebaServo", group="Pushbot")
public class
PruebaServo extends LinearOpMode {

    PruebaServoConfig robot = new PruebaServoConfig();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap , telemetry);
        sleep(1000);
        telemetry.update();

        waitForStart();

        double posicion = 0.5;
        double posicion_2 = 0.5;

        while (opModeIsActive()) {
            telemetry.addLine("Cambia la posicion del servo con los bumpers");

            telemetry.addData("" , "");
            telemetry.addData("Posicion del servo" , posicion);
            telemetry.update();



            if (gamepad1.a) {
               posicion += 0.1;
               posicion_2 = 0.1;
               sleep(300);
            }
            else if (gamepad1.b) {
                posicion -= 0.1;
                posicion_2 = 0.6;
                sleep(300);
            }
            robot.servo.setPosition(posicion);
            robot.servo_2.setPosition(posicion);

        }
    }
}
