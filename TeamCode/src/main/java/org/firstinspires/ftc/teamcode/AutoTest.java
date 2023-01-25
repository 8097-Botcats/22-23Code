package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous(name = "Auto Test")
public class AutoTest extends LinearOpMode {

    double CIRCUMFERENCEOFWHEEL = 298.5; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    Robot robot = new Robot();

    public void runOpMode(){
        OpenCvWebcam webcam;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        robot.init(hardwareMap, telemetry);
        Servo clawServo = hardwareMap.servo.get("clawServo");


        AprilTag pipeline;

        pipeline = new AprilTag();
        webcam.setPipeline(pipeline);

        int[] IDsofInterest = {0, 1, 2};
        int detectionID = 1;
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
        clawServo.setPosition(.25);
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
        clawServo.setPosition(.25);
        telemetry.addData("monke", 1);
        telemetry.update();
        //ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
        waitForStart();
        if (opModeIsActive()) {

        }
    }
}
