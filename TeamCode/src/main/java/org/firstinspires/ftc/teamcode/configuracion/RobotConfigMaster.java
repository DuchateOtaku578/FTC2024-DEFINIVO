package org.firstinspires.ftc.teamcode.configuracion;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
@TeleOp(name= "teleop")
public class RobotConfigMaster extends LinearOpMode {

    public DcMotor enfrenteDer = null; //0

    public DcMotor enfrenteIzq = null; //1
    public DcMotor atrasDer = null; //2
    public DcMotor atrasIzq = null; //


    @Override
    public void runOpMode(){



        enfrenteDer= hardwareMap.get(DcMotor.class, "enfrenteDer");
        enfrenteIzq = hardwareMap.get(DcMotor.class, "enfrenteIzq");
        atrasDer = hardwareMap.get(DcMotor.class, "atrasDer");
        atrasIzq= hardwareMap.get(DcMotor.class, "atrasIzq");

        enfrenteDer.setDirection(DcMotorSimple.Direction.FORWARD);
        enfrenteIzq.setDirection(DcMotorSimple.Direction.FORWARD);
        atrasDer.setDirection(DcMotorSimple.Direction.REVERSE);
        atrasIzq.setDirection(DcMotorSimple.Direction.REVERSE);

        enfrenteDer.setPower(0);
        enfrenteIzq.setPower(0);
        atrasDer.setPower(0);
        atrasIzq.setPower(0);


        enfrenteDer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        enfrenteIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        atrasIzq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

}


