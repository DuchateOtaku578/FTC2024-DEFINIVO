package org.firstinspires.ftc.teamcode.autonomos.pruebas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.configuracion.RobotConfigMaster_RR;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//@Disabled
// Gabo = Arnold
//Beto = úrsula
//Nadia = Olga
//Orión = Petra
@Autonomous(name="Autonomo rojo der M->D Camara", group="Camara")

public class ChasisAutonomo23Camara extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH =1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT =720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength =1280;  // Replace with the focal length of the camera in pixels

    public int randomizacion;


    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot = new RobotConfigMaster_RR();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -62, Math.toRadians(90));
        robot.init(hardwareMap, telemetry);
        sleep(500);
        drive.setPoseEstimate(startPose);
        robot.subirGarra();
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);
        controlHubCam.startStreaming(1280,720);



        TrajectorySequence derecha1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(34,-33, Math.toRadians(180))).addTemporalMarker(0.1, () -> {
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence derecha2 = drive.trajectorySequenceBuilder(derecha1.end())
                .back(5).addTemporalMarker(0.1, () ->{
                    robot.cerrarGarraDer();
                    robot.subirGarra();
                    robot.subirElevador(0.8);
                }).addTemporalMarker(0.1 + 1, () -> {
                    robot.mantenerElevadorBrake();
                })
                .lineToSplineHeading(new Pose2d(51.3, -45, Math.toRadians(0)))
                .build();

        TrajectorySequence derecha3 = drive.trajectorySequenceBuilder(derecha2.end())
                .lineToSplineHeading(new Pose2d(18,-8, Math.toRadians(180))).addTemporalMarker(0.1, () ->{
                    robot.cerrarGarraDer();
                    robot.cerrarGarraIzq();
                })
                .lineToConstantHeading(new Vector2d(-30, -8))
                .lineToConstantHeading(new Vector2d(-55, -11.5)).addTemporalMarker(5, () ->{
                    robot.bajarGarra();
                    robot.abrirGarraDer();
                    robot.subirElevador(0.7);
                }).addTemporalMarker(5 + 0.6, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(5)
                .build();

        TrajectorySequence derecha4 = drive.trajectorySequenceBuilder(derecha3.end())
                .lineToLinearHeading(new Pose2d(-35 ,-5, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(10, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(49, -37), Math.toRadians(0)).addTemporalMarker(3.5, () ->{
                    robot.subirElevador(0.8);
                }).addTemporalMarker(3.2 + 1.6, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence derecha5 = drive.trajectorySequenceBuilder(derecha4.end())
                .back(5).addTemporalMarker(0.1, () ->{
                    robot.bajarElevadorSinEncoder(0.8);
                }).addTemporalMarker(0.1 + 0.9, () ->{
                    robot.mantenerElevadorBrake();
                })
                .strafeLeft(25)
                .forward(5)
                .build();

        TrajectorySequence izquierda1 = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(15,-34.3, Math.toRadians(180))).addTemporalMarker(0.3, () -> {
                    robot.bajarGarra();
                })
                .forward(5.5)
                .build();

        TrajectorySequence izquierda2 = drive.trajectorySequenceBuilder(izquierda1.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(52 , -29
                        , Math.toRadians(0))).addTemporalMarker(0.1,() ->{
                    robot.subirGarra();
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.1 + 1.1 , () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence izquierda3 = drive.trajectorySequenceBuilder(izquierda2.end())
                .lineToSplineHeading(new Pose2d(18,-8, Math.toRadians(180))).addTemporalMarker(0.1, () ->{
                    robot.cerrarGarraDer();
                    robot.cerrarGarraIzq();
                })
                .lineToConstantHeading(new Vector2d(-30, -8))
                .lineToConstantHeading(new Vector2d(-55, -11.5)).addTemporalMarker(5, () ->{
                    robot.bajarGarra();
                    robot.abrirGarraDer();
                    robot.subirElevador(0.7);
                }).addTemporalMarker(5 + 0.6, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(5)
                .build();

        TrajectorySequence izquierda4 = drive.trajectorySequenceBuilder(izquierda3.end())
                .lineToLinearHeading(new Pose2d(-35 ,-5, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(10, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(49, -37), Math.toRadians(0)).addTemporalMarker(3.5, () ->{
                    robot.subirElevador(0.8);
                }).addTemporalMarker(3.2 + 1.6, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence izquierda5 = drive.trajectorySequenceBuilder(izquierda4.end())
                .back(5).addTemporalMarker(0.1, () ->{
                    robot.bajarElevadorSinEncoder(0.8);
                }).addTemporalMarker(0.1 + 0.9, () ->{
                    robot.mantenerElevadorBrake();
                })
                .strafeLeft(25)
                .forward(5)
                .build();

        TrajectorySequence centro1 = drive.trajectorySequenceBuilder(startPose)
                .forward(28)
                .addTemporalMarker(0.3, () ->{
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence centro2 = drive.trajectorySequenceBuilder(centro1.end())
                .back(10)
                .lineToSplineHeading(new Pose2d(52, -37.2, Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.subirGarra();
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.1 + 1.2, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence centro3 = drive.trajectorySequenceBuilder(centro2.end())
                .lineToSplineHeading(new Pose2d(18,-8, Math.toRadians(180))).addTemporalMarker(0.1, () ->{
                    robot.cerrarGarraDer();
                    robot.cerrarGarraIzq();
                })
                .lineToConstantHeading(new Vector2d(-30, -8))
                .lineToConstantHeading(new Vector2d(-55, -12)).addTemporalMarker(5, () ->{
                    robot.bajarGarra();
                    robot.abrirGarraDer();
                    robot.subirElevador(0.7);
                }).addTemporalMarker(5 + 0.6, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(5)
                .build();

        TrajectorySequence centro4 = drive.trajectorySequenceBuilder(centro3.end())
                .lineToLinearHeading(new Pose2d(-35 ,-5, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(10, -5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(49, -37), Math.toRadians(0)).addTemporalMarker(3.5, () ->{
                    robot.subirElevador(0.8);
                }).addTemporalMarker(3.2 + 1.6, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence centro5 = drive.trajectorySequenceBuilder(centro4.end())
                .back(5).addTemporalMarker(0.1, () ->{
                    robot.bajarElevadorSinEncoder(0.8);
                }).addTemporalMarker(0.1 + 0.9, () ->{
                    robot.mantenerElevadorBrake();
                })
                .strafeLeft(25)
                .forward(5)
                .build();










        while(opModeInInit()) {
        if (getDistance(width) >= 20 && getDistance(width) <27){
            telemetry.addLine("Randomizacion: derecha");
        }else
        if(getDistance(width) >= 30 && getDistance(width) <= 38){
            telemetry.addLine("Randomizacion: centro");
        }else
            telemetry.addLine("Randomizacion: izquierda");

        telemetry.update();
    }
        waitForStart();

        if (!isStopRequested()) {
            if (getDistance(width) >= 20 && getDistance(width) <27){
                controlHubCam.stopStreaming();
                controlHubCam.closeCameraDevice();

                drive.followTrajectorySequence(derecha1);
                robot.abrirGarraDer();
                sleep(400);
                drive.followTrajectorySequence(derecha2);
                robot.abrirGarraIzq();
                sleep(200);
                robot.bajarElevadorSinEncoder(0.7);
                sleep(800);
                robot.mantenerElevadorBrake();
                drive.followTrajectorySequence(derecha3);
                robot.cerrarGarraDer();
                sleep(200);
                robot.subirGarra();
                sleep(200);
                robot.bajarElevador(0.6);
                sleep(500);
                robot.mantenerElevadorBrake();
                drive.followTrajectorySequence(derecha4);
                robot.abrirGarraDer();
                sleep(200);
                drive.followTrajectorySequence(derecha5);




            } else {
                if(getDistance(width) >= 30 && getDistance(width) <= 38
                ){
                    controlHubCam.stopStreaming();
                    controlHubCam.closeCameraDevice();
                    drive.followTrajectorySequence(centro1);
                    robot.abrirGarraDer();
                    sleep(200);
                    drive.followTrajectorySequence(centro2);
                    robot.abrirGarraIzq();
                    sleep(200);
                    robot.bajarElevadorSinEncoder(0.7);
                    sleep(800);
                    robot.mantenerElevadorBrake();
                    drive.followTrajectorySequence(centro3);
                    robot.cerrarGarraDer();
                    sleep(200);
                    robot.subirGarra();
                    sleep(200);
                    robot.bajarElevador(0.6);
                    sleep(500);
                    robot.mantenerElevadorBrake();
                    drive.followTrajectorySequence(centro4);
                    robot.abrirGarraDer();
                    sleep(200);
                    drive.followTrajectorySequence(centro5);



                } else {
                    controlHubCam.stopStreaming();
                    controlHubCam.closeCameraDevice();
                    drive.followTrajectorySequence(izquierda1);
                    robot.abrirGarraDer();
                    sleep(200);
                    drive.followTrajectorySequence(izquierda2);
                    robot.abrirGarraIzq();
                    sleep(200);
                    robot.bajarElevadorSinEncoder(0.7);
                    sleep(800);
                    robot.mantenerElevadorBrake();
                    drive.followTrajectorySequence(izquierda3);
                    robot.cerrarGarraDer();
                    sleep(200);
                    robot.subirGarra();
                    sleep(200);
                    robot.bajarElevador(0.6);
                    sleep(500);
                    robot.mantenerElevadorBrake();
                    drive.followTrajectorySequence(izquierda4);
                    robot.abrirGarraDer();
                    sleep(200);
                    drive.followTrajectorySequence(izquierda5);





                }

            }
        }


    }
    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance

        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new YellowBlobDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);

    }
    public class YellowBlobDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect yellow regions
            Mat yellowMask = preprocessFrame(input);

            // Find contours of the detected yellow regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(yellowMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find the largest yellow contour (blob)
            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                // Draw a red outline around the largest detected object
                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(255, 0, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(largestContour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);

            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerYellow = new Scalar(100, 100, 100);
            Scalar upperYellow = new Scalar(180, 255, 255 );



            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }
}
