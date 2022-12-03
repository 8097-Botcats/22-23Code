package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.BasicDiagnosticFormatter;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "Red Left Auto")
public class RedLeftAuto extends LinearOpMode {
    public void runOpMode() {
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        AprilTag pipeline;

        pipeline = new AprilTag();
        webcam.setPipeline(pipeline);

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
        Pose2d startPose = new Pose2d(-35.5, -60.5, Math.toRadians(90));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajSeqLeft = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-59, -60.5))
                .lineTo(new Vector2d(-59, -34))
                .build();

        TrajectorySequence trajSeqCenter = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-35.5, -34))
                .build();

        TrajectorySequence trajSeqRight = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-12, -60.5))
                .lineTo(new Vector2d(-12, -34))
                .build();

        while(!isStopRequested() || !isStarted()){
            ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
            for(AprilTagDetection detection : detections){
                detectionID = detection.id;
            }
        }

        if (!isStopRequested()) {
            if(detectionID == 0){
                drive.followTrajectorySequence(trajSeqLeft);
            }
            if(detectionID == 1){
                drive.followTrajectorySequence(trajSeqCenter);
            }
            if(detectionID == 2){
                drive.followTrajectorySequence(trajSeqRight);
            }

        }

    }
}
