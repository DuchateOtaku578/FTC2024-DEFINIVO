package org.firstinspires.ftc.teamcode.test.imu;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * HARDWARE TEMPLATE
 * Usa este template para:
 * -Inicializar tus motores y servos
 * -Sentido de giro de los motores
 * -Modo de uso de los motores
 * <p>
 * *
 */

public class PruebaSensorIMUConfig {
    /**
     * Declaracion de los motores/servo -- modificar
     */
    //Declarar objetos (motores y servos), es recomendable usar el mismo nombre
    //para el objeto y en la configracion en el robot




    BNO055IMU imu;

    /* local OpMode members. -- no modificar */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor -- no modificar */
    public PruebaSensorIMUConfig() {

    }

    /**
     * Inicializar hardware --modificar
     */
    public void init(HardwareMap ahwMap, Telemetry telemetry) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        telemetry.addLine("Motores inicializados...");
        // Definir e inicializar hardware
        /*En las comillas (deviceName) debe de ir el nombre que hayas puesto en la configuracion
        del robot*/




        imu = hwMap.get(BNO055IMU.class , "imu");

        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();

        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        IMUParameters.calibrationDataFile = "BNO055IMU.json";

        imu.initialize(IMUParameters);


        /** Fin de la configuracion*/

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
