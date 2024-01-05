package org.firstinspires.ftc.teamcode.test.imu;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.domain.Chasis;

@Autonomous(name="Prueba IMU", group = "PushBot")
@Disabled
public class PruebaSensorIMU extends LinearOpMode {
    @Override
    public void runOpMode(){
        PruebaSensorIMUConfig robot = new PruebaSensorIMUConfig();
        robot.init(hardwareMap,telemetry);
        Chasis chasis = new Chasis(robot.imu);

        telemetry.addLine("sensor capacitado");

        waitForStart();

        while(opModeIsActive()){
            
            telemetry.addData("Orientacion angular", chasis.obtenerAngulo());
            telemetry.update();

        }
    }
}
