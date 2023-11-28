package org.firstinspires.ftc.teamcode.test.servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="PruebaServo", group="Pushbot")
// @Disabled
public class PruebaServo extends LinearOpMode {

    PruebaServoConfig robot = new PruebaServoConfig();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap , telemetry);
        sleep(1000);
        telemetry.update();

        waitForStart();

        double posicion = 0;
        double posicion_2 = 1;

        while (opModeIsActive()) {
            telemetry.addLine("Cambia la posicion del servo con los bumpers");

            telemetry.addData("" , "");
            telemetry.addData("Posicion del servo" , posicion);
            telemetry.addData("Posicion del servo_2", posicion_2);

            telemetry.addLine("Ever es gay");
            telemetry.update();



            if (gamepad1.a) {
               posicion += 0.1;
               posicion_2 -= 0.1;
               sleep(300);
            }
            else if (gamepad1.b) {
                posicion -= 0.1;
                posicion_2 += 0.1;
                sleep(300);
            }




            robot.servo.setPosition(posicion);
            robot.servo_2.setPosition(posicion_2);
        }
    }
}
