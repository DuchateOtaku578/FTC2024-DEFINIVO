package org.firstinspires.ftc.teamcode.configuracion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotConfigMaster_RR {

    public DcMotor elevador_1;

    public DcMotor elevador_2;

    public Servo servoGarra;

    public Servo servoDerecha;

    public Servo servoIzquierda;


    public DcMotor gancho;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public double MID_POS = 0.5;

    public RobotConfigMaster_RR(){

    }

    public void init(HardwareMap ahwMap, Telemetry telemetry){

        hwMap  = ahwMap;

        elevador_1 = hwMap.get(DcMotor.class, "elevador_1");
        elevador_2 = hwMap.get(DcMotor.class, "elevador_2");
        servoGarra = hwMap.get(Servo.class, "Garra");
        servoDerecha = hwMap.get(Servo.class, "servoDer");
        servoIzquierda = hwMap.get(Servo.class, "servoIzq");
        gancho = hwMap.get(DcMotor.class, "Gancho");

        derecho(elevador_1, gancho);
        reversa(elevador_2);


        elevador_1.setPower(0);
        elevador_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador_2.setPower(0);
        elevador_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        gancho.setPower(0);
        gancho.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoGarra.setPosition(MID_POS);
        //servoDerecha.setPosition(Servo.MIN_POSITION);
        servoIzquierda.setPosition(Servo.MAX_POSITION);

        elevador_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gancho.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

    public void usarSetZeroPosition(DcMotor... motores){
        for(DcMotor motor : motores){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void subirElevador(){
        usarUsingEncoder(elevador_2, elevador_1);
        elevador_1.setPower(1);
        elevador_2.setPower(1);
    }

    public void bajarElevador(){
        usarUsingEncoder(elevador_1,elevador_2);
        elevador_1.setPower(-1);
        elevador_2.setPower(-1);
    }
    public void mantenerElevador(){
        elevador_1.setTargetPosition(elevador_1.getCurrentPosition());
        elevador_2.setTargetPosition(elevador_2.getCurrentPosition());
        usarRunToPosition(elevador_1, elevador_2);
        elevador_1.setPower(1);
        elevador_2.setPower(1);
    }

    public void mantenerElevadorBrake(){
        elevador_1.setPower(0);
        elevador_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador_2.setPower(0);
        elevador_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void bajarGarra(){
        servoDerecha.setPosition(0.425);
        servoIzquierda.setPosition(0.425);
    }

    public void subirGarra(){
        servoDerecha.setPosition(1);
        servoIzquierda.setPosition(1);
    }

    public void abrirGarra(){
        servoGarra.setPosition(0.3);
    }

    public void cerrarGarra(){
        servoGarra.setPosition(0.1);
    }

    public void enrollarGancho() {
        gancho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gancho.setPower(1);
    }

    public void mantenerGancho() {
        gancho.setTargetPosition(gancho.getCurrentPosition());

        gancho.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gancho.setPower(1);
    }

    public void desenrrollarGancho(){
        gancho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gancho.setPower(-1);
    }
}
