package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class RedLeftAuto extends LinearOpMode {
    public void runOpMode() throws InterruptedException{

        Pose2d startPose = new Pose2d(0, 0, 0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-12, -60.5, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-10, -10, Math.toRadians(45)))
                .build()

        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
        }

    }
}
