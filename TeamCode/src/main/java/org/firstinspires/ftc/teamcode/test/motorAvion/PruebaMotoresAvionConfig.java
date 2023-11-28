package org.firstinspires.ftc.teamcode.test.motorAvion;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PruebaMotoresAvionConfig {
    public DcMotor motor_1;
    public DcMotor motor_2;

    public Servo servo;





    HardwareMap hwMap;
    private ElapsedTime period = new ElapsedTime();

    public void init(HardwareMap ahwMap, Telemetry telemetry){
        hwMap = ahwMap;

        motor_1 = hwMap.get(DcMotor.class, "motor");
        motor_2 = hwMap.get(DcMotor.class,"motor_2");
        servo = hwMap.get(Servo.class,"comosea");

        derecho(motor_1);
        reversa(motor_2);

        motor_1.setPower(0);
        motor_2.setPower(0);
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
    public void encender(){
        motor_1.setPower(1);
        motor_2.setPower(1);
    }
    public void parar(){
        motor_1.setPower(0);
        motor_2.setPower(0);
    }

    public void irPosicionInicial(){
        servo.setPosition(0);
    }
    public void moverServo(){

        servo.setPosition(0.5);
    }
    public void irPosicionMaxima(){
        servo.setPosition(1);
    }
}
