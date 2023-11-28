package org.firstinspires.ftc.teamcode.configuracion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotConfigMaster_3 {

    //CONTROL HUB

    //EXPANSION HUB
    public DcMotor enfrenteDer = null; //0
    public DcMotor enfrenteIzq = null; //1
    public DcMotor atrasDer = null; //2
    public DcMotor atrasIzq = null; //3

    public DcMotor[] motores = {enfrenteDer,enfrenteIzq, atrasDer, atrasIzq};





    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public RobotConfigMaster_3() {

    }

    public void init(HardwareMap ahwMap, Telemetry telemetry) {

        hwMap = ahwMap;

        motores[0] = hwMap.get(DcMotor.class, "enfrenteDer");
        motores[1] = hwMap.get(DcMotor.class, "enfrenteIzq");
        motores[2] = hwMap.get(DcMotor.class, "atrasDer");
        motores[3]= hwMap.get(DcMotor.class, "atrasIzq");

        enfrenteDer = motores[0];
        enfrenteIzq = motores[1];
        atrasDer = motores[2];
        atrasIzq = motores[3];


        telemetry.addLine("Motores inicializados...");
        telemetry.update();





        telemetry.addLine("Sensores inicializados...");
        telemetry.update();


        reversa(atrasIzq , enfrenteIzq);
        derecho(atrasDer , enfrenteDer);

        telemetry.addLine("Cambio de giro de motores hecho...");
        telemetry.update();

        enfrenteDer.setPower(0);
        enfrenteIzq.setPower(0);
        atrasDer.setPower(0);
        atrasIzq.setPower(0);
        telemetry.addLine("Motores al 0%...");
        telemetry.update();

        usarWithoutEncoder(enfrenteDer, enfrenteIzq, atrasDer, atrasIzq);

        telemetry.addLine("Motores configurados...");
        telemetry.update();

        telemetry.addData("Hardware", "Inicializado");
        telemetry.update();
    }







    public void reversa(DcMotor... motores) {
        for (DcMotor motor : motores) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void derecho(DcMotor... motores) {
        for (DcMotor motor : motores) {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void usarWithoutEncoder(DcMotor... motores) {
        for (DcMotor motor : motores) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void usarUsingEncoder(DcMotor... motores) {
        for (DcMotor motor : motores) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void usarRunToPosition(DcMotor... motores) {
        for (DcMotor motor : motores) {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }



}


