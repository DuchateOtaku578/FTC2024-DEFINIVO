package org.firstinspires.ftc.teamcode.test.odometria;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class ChasisOdometriaConfig {
    //Chasis e IMU
    public DcMotor enfrenteDer = null; //0
    public DcMotor enfrenteIzq = null; //1
    public DcMotor atrasDer = null; //2
    public DcMotor atrasIzq = null; //3x

    public DcMotor[] motores = {enfrenteDer,enfrenteIzq, atrasDer, atrasIzq};

    public BNO055IMU imu;

    //Encoders de la oodmetria
    public DcMotor encoderDer;
    public DcMotor encoderIzq;
    public DcMotor encoderTheta;

    //Calculos de Odometria

    public final static double encoderTicksPerRevolution = 8192; //ticks por revolucion del encoder

    public final static double wheelRadius = 2.31; //radius wheel in centimeretrs

    public final static double large =43.2; //distancia entre los dos encoders paralelos (pendiente)

    public final static double middlePoint = 21.6; //distancia entre el punto central de los 2 encoders y el encoder Delta (pendiente)

    public final static double cm_per_tick = 2.0 * Math.PI * wheelRadius/encoderTicksPerRevolution; //distancia que recorre un encoder por tick en centimetro

    public double currentRightTicks;
    public double currentLeftTIcks;
    public double currentDeltaTicks;

    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    public ChasisOdometriaConfig() {

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

        //igualamos los encoders a los motores


        encoderDer = enfrenteDer;
        encoderIzq = enfrenteIzq;
        encoderTheta = atrasDer;

        reversa(atrasIzq , enfrenteIzq);
        derecho(atrasDer , enfrenteDer);
        frenarMotores(enfrenteDer, enfrenteIzq, atrasDer, atrasIzq);
        usarWithoutEncoder(enfrenteDer, enfrenteIzq, atrasDer, atrasIzq);
        resetDriveEncoders();
        parar();

        //USAR EL IMU DEL EXPANSION HUB IMPORTANTE
        imu = hwMap.get(BNO055IMU.class , "imu");

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();

        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMUParameters.calibrationDataFile = "BNO055IMU.json";

        imu.initialize(IMUParameters);


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

    public void frenarMotores(DcMotor... motores){
        for(DcMotor motor : motores){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public double   obtenerAngulo(){
        Orientation angulos;
        angulos = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
        double primerAngulo = angulos.firstAngle;
        primerAngulo += 180;
        return primerAngulo;
    }

    public void resetDriveEncoders(){
        enfrenteDer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enfrenteDer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        enfrenteIzq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        enfrenteIzq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        atrasDer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasDer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        atrasIzq.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        atrasIzq.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void parar(){
        enfrenteDer.setPower(0);
        enfrenteIzq.setPower(0);
        atrasDer.setPower(0);
        atrasIzq.setPower(0);
    }




}



