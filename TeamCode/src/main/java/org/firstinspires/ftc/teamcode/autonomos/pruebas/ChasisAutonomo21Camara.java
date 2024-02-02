package org.firstinspires.ftc.teamcode.autonomos.pruebas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
@Autonomous(name="Autonomo rojo izq M->D Camara", group="Pushbot")

public class ChasisAutonomo21Camara extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 720    ;  // Replace with the focal length of the camera in pixels

    public int randomizacion;


    @Override
    public void runOpMode() {
        RobotConfigMaster_RR robot = new RobotConfigMaster_RR();
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-37, -62, Math.toRadians(90));
        robot.init(hardwareMap, telemetry);
        sleep(500);
        drive.setPoseEstimate(startPose);
        robot.subirGarra();
        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);



        Pose2d nadia = drive.getPoseEstimate();
        Pose2d uwu = new Pose2d(nadia.getX(), nadia.getY(), nadia.getHeading());


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-42.5, -45))
                .build();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq.end())
                .forward(9.5)
                .strafeRight(11.5)
                .build();


        //Izquierda Detectada
        TrajectorySequence trajSeq1 = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(-47, -15, Math.toRadians(270  - 1e-6))).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq1.end())
                .back(16)
                .lineToLinearHeading(new Pose2d(38,-12, Math.toRadians(0))).addTemporalMarker(0.1, () ->{
                    robot.subirGarra();
                    robot.cerrarGarraDer();
                })
                .splineToConstantHeading(new Vector2d(50, -28.7), Math.toRadians(0))
                .build();

        //Centro Detectado
        TrajectorySequence trajSeq4 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .strafeLeft(10)
                .lineToSplineHeading(new Pose2d(-39, -14.5, Math.toRadians(270)))
                .build();

        TrajectorySequence trajSeq5 = drive.trajectorySequenceBuilder(trajSeq4.end())
                .back(15).addTemporalMarker(0.1 , () ->{
                    robot.subirGarra();
                    robot.cerrarGarraDer();
                })
                .lineToLinearHeading(new Pose2d(30, -12, Math.toRadians(0)))
                .build();

        //Derecho
        TrajectorySequence trajSeq6 = drive.trajectorySequenceBuilder(trajSeq3.end())
                .splineToLinearHeading(new Pose2d(-55, -34,Math.toRadians(0)), Math.toRadians(90))
                .splineTo(new Vector2d(-35, -33), Math.toRadians(0)).addTemporalMarker(0.1, () ->{
                    robot.bajarGarra();
                })
                .build();

        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(trajSeq6.end())
                .splineToConstantHeading(new Vector2d(-45, -33), Math.toRadians(0)).addTemporalMarker(0.1, () ->{
                    robot.subirGarra();
                    robot.cerrarGarraDer();
                })
                .splineToConstantHeading(new Vector2d(-34, 0),Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(30, -12), Math.toRadians(0))
                .build();

//+ 0.7-0.8 temporal marker
        TrajectorySequence trayectoria11 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .back(10)
                .strafeRight(29.5).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.8);
                }).addTemporalMarker(0.1 + 0.8, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(14)
                .build();

        TrajectorySequence trayectoria12 = drive.trajectorySequenceBuilder(trajSeq5.end())
                .splineToConstantHeading(new Vector2d(50, -35.25
                ),Math.toRadians(0))
                .build();

        TrajectorySequence trayectoria13 = drive.trajectorySequenceBuilder(trayectoria12.end())
                .back(10)
                .strafeRight(22.5).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.7);
                }).addTemporalMarker(0.1 + 0.92, () ->{
                    robot.mantenerElevadorBrake();
                })
                .forward(14)
                .build();

        TrajectorySequence trayectoria14 = drive.trajectorySequenceBuilder(trajSeq7.end())
                .splineToConstantHeading(new Vector2d(50, -43),Math.toRadians(0)).addTemporalMarker(0.1, () ->{
                    robot.subirElevador(0.7);
                }).addTemporalMarker(0.1 + 1.1, () ->{
                    robot.mantenerElevadorBrake();
                })
                .build();

        TrajectorySequence trayectorira15 = drive.trajectorySequenceBuilder(trayectoria14.end())
                .back(10).addTemporalMarker(0.1, () ->{
                    robot.bajarElevador(0.8);
                }).addTemporalMarker(0.1 + 0.8, () ->{
                    robot.mantenerElevadorBrake();
                    robot.cerrarGarraIzq();
                })
                .strafeRight(18)
                .forward(15)
                .build();
    while(opModeInInit()) {
        if (getDistance(width) >= 12 && getDistance(width) < 15){
            randomizacion = 1;
        }else
        if(getDistance(width) >=15 && getDistance(width) <= 20){
            randomizacion = 2;
        }else
            randomizacion = 3;

        telemetry.addData("Randomizacion: ", randomizacion);

        telemetry.update();
    }
        waitForStart();

        if (!isStopRequested()) {
            if (getDistance(width) >= 12 && getDistance(width) < 15){
                drive.followTrajectorySequence(trajSeq1);
                robot.abrirGarraDer();
                sleep(300);
                drive.followTrajectorySequence(trajSeq2);
                robot.subirElevador(0.7);
                sleep(1200);
                robot.mantenerElevadorBrake();
                robot.abrirGarraIzq();
                sleep(200);
                drive.followTrajectorySequence(trayectoria11);
            } else {
                if(getDistance(width) >=15 && getDistance(width) <= 20){
                    drive.followTrajectorySequence(trajSeq4);
                    robot.bajarGarra();
                    robot.abrirGarraDer();
                    sleep(500);
                    drive.followTrajectorySequence(trajSeq5);
                    drive.followTrajectorySequence(trayectoria12);
                    robot.subirElevador(0.7);
                    sleep(1000);
                    robot.mantenerElevadorBrake();
                    robot.abrirGarraIzq();
                    sleep(850);
                    drive.followTrajectorySequence(trayectoria13);

                } else {
                    drive.followTrajectorySequence(trajSeq6);
                    robot.abrirGarraDer();
                    sleep(400);
                    drive.followTrajectorySequence(trajSeq7);
                    drive.followTrajectorySequence(trayectoria14);
                    robot.abrirGarraIzq();
                    sleep(500);
                    drive.followTrajectorySequence(trayectorira15);


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
