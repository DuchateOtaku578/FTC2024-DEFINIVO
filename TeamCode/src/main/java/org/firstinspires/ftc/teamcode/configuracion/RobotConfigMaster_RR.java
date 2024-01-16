package org.firstinspires.ftc.teamcode.configuracion;

import com.qualcomm.ftccommon.configuration.EditServoListActivity;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class RobotConfigMaster_RR {

    public DcMotor elevador_1;

    public DcMotor elevador_2;

    public Servo servoDerecha;

    public Servo servoIzquierda;

    public DcMotor gancho;

    public Servo ligaAvion;

    public Servo pinzaDer;

    public Servo pinzaIzq;


    public DistanceSensor distanceSensor;

    public DistanceSensor distanceSensor_2;
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public double MID_POS = 0.5;

    public RobotConfigMaster_RR(){

    }

    public void init(HardwareMap ahwMap, Telemetry telemetry){

        hwMap  = ahwMap;
        //elevador
        elevador_1 = hwMap.get(DcMotor.class, "elevador_1");
        elevador_2 = hwMap.get(DcMotor.class, "elevador_2");
        distanceSensor = hwMap.get(DistanceSensor.class, "sensor_distancia");
        distanceSensor_2 = hwMap.get(DistanceSensor.class, "sensor_distancia_2");
        //garra
        pinzaDer = hwMap.get(Servo.class,"pinzaDer");
        pinzaIzq = hwMap.get(Servo.class,"pinzaIzq");
        servoDerecha = hwMap.get(Servo.class, "servoDer");
        servoIzquierda = hwMap.get(Servo.class, "servoIzq");
        //gancho
        gancho = hwMap.get(DcMotor.class, "Gancho");
        //avion
        ligaAvion = hwMap.get(Servo.class, "liga");

        derecho(elevador_1 /*,gancho*/);
        reversa(elevador_2);

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distanceSensor;
        Rev2mDistanceSensor sensorTImeOfFlight_2 = (Rev2mDistanceSensor) distanceSensor_2;


        cerrarGarraIzq();
        cerrarGarraDer();


        elevador_1.setPower(0);
        elevador_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador_2.setPower(0);
        elevador_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ligaAvion.setPosition(0.8);

        elevador_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevador_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gancho.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        usarUsingEncoder(elevador_1,elevador_2);

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

    public void subirElevador(double potencia){
        usarUsingEncoder(elevador_2, elevador_1);
        elevador_1.setPower(potencia);
        elevador_2.setPower(potencia);
    }

    public void subirElevador(double potencia, int pulsos){
        elevador_1.setTargetPosition(-pulsos);
        elevador_2.setTargetPosition(-pulsos);
        usarRunToPosition(elevador_1,elevador_2);
        elevador_1.setPower(1);
        elevador_2.setPower(1);

    }

    public void bajarElevador(double potencia, int pulsos){
        elevador_1.setTargetPosition(pulsos);
        elevador_2.setTargetPosition(pulsos);
        usarRunToPosition(elevador_1,elevador_2);
        elevador_1.setPower(-potencia);
        elevador_2.setPower(-potencia);
    }

    public void bajarElevador(double potencia){
        usarUsingEncoder(elevador_1,elevador_2);
        elevador_1.setPower(-potencia);
        elevador_2.setPower(-potencia);
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
        servoDerecha.setPosition(0.63);
        servoIzquierda.setPosition(0.63);
    }

    public void subirGarra(){
        servoDerecha.setPosition(0.85);
        servoIzquierda.setPosition(0.85);
    }

    public void enrollarGancho() {
        gancho.setPower(1);
    }

    public void mantenerGancho() {
        gancho.setPower(0);
        gancho.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
    }

    public void desenrrollarGancho(){
        gancho.setPower(-1);
    }

    public void cerrarGarraDer(){

        pinzaDer.setPosition(0.74);
    }

    public void cerrarGarraIzq(){
        pinzaIzq.setPosition(0.3);
    }

    public void abrirGarraDer(){
        pinzaDer.setPosition(0.5);
    }

    public void abrirGarraIzq(){
        pinzaIzq.setPosition(0.5);
    }


    public double distanciaCentimetros(){
        double cm = distanceSensor.getDistance(DistanceUnit.CM);
        return cm;
    }
    public double distanciaMetros(){
        double m = distanceSensor.getDistance(DistanceUnit.METER);
        return m;
    }
    public double distanciaMilimetros(){
        double mm = distanceSensor.getDistance(DistanceUnit.MM);
        return mm;
    }

    public double distanciaCentimetros_2(){
        double cm2 = distanceSensor_2.getDistance(DistanceUnit.CM);
        return cm2;
    }



}
