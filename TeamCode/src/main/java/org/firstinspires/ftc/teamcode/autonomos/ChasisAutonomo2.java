package org.firstinspires.ftc.teamcode.autonomos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//@Disabled
@Autonomous(name="ChasisAutonomo2", group="Pushbot")

public class ChasisAutonomo2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(30, 0, Math.toRadians(90)))
                .turn(Math.toRadians(90))
                .forward(30)
                .lineToLinearHeading(new Pose2d(-30,0,Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .forward(30)
                .build();



        telemetry.update();

        waitForStart();

        if(!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);

        }

    }
}
