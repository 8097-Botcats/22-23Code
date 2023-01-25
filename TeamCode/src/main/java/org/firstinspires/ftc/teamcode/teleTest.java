package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//TELEOP WITH 1 CONTROLLER
@TeleOp(name = "Tele Test")
public class teleTest extends LinearOpMode {
    public void runOpMode() {

        DcMotor lift1 = hardwareMap.dcMotor.get("liftMotor1");
        DcMotor lift2 = hardwareMap.dcMotor.get("liftMotor2");

        double driveSpeed = .5;
        boolean aButton = true;
        double finalAngle;

        boolean lift1run = true;

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if((gamepad1.right_trigger > 0) && gamepad1.left_trigger == 0){
                lift1.setPower(gamepad1.right_trigger);
                lift2.setPower(-gamepad1.right_trigger);
            }
            if((gamepad1.left_trigger > 0) && gamepad1.right_trigger == 0){
                lift1.setPower(-gamepad1.left_trigger);
                lift2.setPower(gamepad1.left_trigger);
            }
            if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                lift1.setPower(0);
                lift2.setPower(0);
            }

            telemetry.addData("lift1 enc ticks", lift1.getCurrentPosition());
            telemetry.addData("lift2 enc ticks", lift2.getCurrentPosition());
            telemetry.update();

            /*if (gamepad1.a && aButton) {
                aButton = false;
                lift1run = !lift1run;
            }
            if (!gamepad1.a && !aButton) {
                aButton = true;
            }
            if(lift1run){
                if((gamepad1.right_trigger > 0) && gamepad1.left_trigger == 0){
                    lift1.setPower(-gamepad1.right_trigger);
                }
                if((gamepad1.left_trigger > 0) && gamepad1.right_trigger == 0){
                    lift1.setPower(gamepad1.left_trigger);
                }
                if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                    lift1.setPower(0);
                }
            }
            if(!lift1run){
                if((gamepad1.right_trigger > 0) && gamepad1.left_trigger == 0){
                    lift2.setPower(-gamepad1.right_trigger);
                }
                if((gamepad1.left_trigger > 0) && gamepad1.right_trigger == 0){
                    lift2.setPower(gamepad1.left_trigger);
                }
                if(gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                    lift2.setPower(0);
                }
            }
            telemetry.addData("lift1run", lift1run);
            telemetry.update();*/

        }
    }
}
