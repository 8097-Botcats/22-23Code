package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Red Left Auto")
public class RedLeftAuto extends LinearOpMode {
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0, 0, 0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12, -60.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)))
                .build();

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }

    }
}
