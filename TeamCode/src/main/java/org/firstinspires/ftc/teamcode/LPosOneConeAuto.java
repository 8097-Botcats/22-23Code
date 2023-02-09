package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "Left 1 Cone Auto")
public class LPosOneConeAuto extends LinearOpMode {
    public void runOpMode() {
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        AprilTag pipeline;

        pipeline = new AprilTag();
        webcam.setPipeline(pipeline);

        Robot robot = new Robot();

        int[] IDsofInterest = {0, 1, 2};
        int detectionID = 0;
        boolean tagFound = false;


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(90));
        Pose2d postScorePose = new Pose2d(-12, -36, Math.toRadians(90));
        //Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        robot.init(hardwareMap, telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //robot.init(hardwareMap, telemetry);
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeqScore = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(.5)
                .forward(1)
                .lineTo(new Vector2d(-8.5, -63.5))
                .lineTo(new Vector2d(-8.5, -34))
                .turn(Math.toRadians(-40.4))
                .addTemporalMarker(9, () -> {
                    robot.lift(.6, 4500);
                    robot.liftPower(.0275);
                })
                .waitSeconds(3)
                //.lineTo(new Vector2d(-66, -38))
                .forward(4)
                .waitSeconds(2)
                .addTemporalMarker(13, () -> {
                    robot.openClaw();
                })
                .lineToLinearHeading(postScorePose)
                .build();

        Trajectory traj0 = drive.trajectoryBuilder(postScorePose)
                .strafeLeft(48)
                .build();

        Trajectory traj1 = drive.trajectoryBuilder(postScorePose)
                .strafeLeft(24)
                .build();
        while(!isStopRequested() && !isStarted()) {
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            for (AprilTagDetection detection : detections) {
                detectionID = detection.id;
                telemetry.addData("DetectionID", detectionID);
                telemetry.addData("detection.id", detection.id);
                telemetry.update();
                sleep(50);
            }
        }

        waitForStart();
        if (opModeIsActive()) {
            robot.closeClaw();
            sleep(500);
            robot.lift(.5, 100);
            if(detectionID == 0){
                drive.followTrajectorySequence(trajSeqScore);
                robot.lift(-.7, -3900);
                drive.followTrajectory(traj0);
            }
            if(detectionID == 1){
                drive.followTrajectorySequence(trajSeqScore);
                robot.lift(-.7, -3900);
                drive.followTrajectory(traj1);
            }
            if(detectionID == 2){
                drive.followTrajectorySequence(trajSeqScore);
                robot.lift(-.7, -3900);
            }
            //drive.followTrajectory(trajLeft);
            //drive.followTrajectory(trajForward);
            //drive.followTrajectorySequence(trajSeqLeft);

        }

    }
}
