package org.firstinspires.ftc.teamcode;



import static android.os.SystemClock.currentThreadTimeMillis;
import static android.os.SystemClock.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.*;

public class Robot {
    /* Public OpMode members. */
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor bl = null;
    DcMotor br = null;
    DcMotor lift1 = null;
    DcMotor lift2 = null;

    Servo clawServo = null;

    double CIRCUMFERENCEOFWHEEL = 314.159; //mm
    double ENCODERTICKS = 537.7;
    double GEARRATIO = 1;
    double TICKSTOMMTRAVEL = (CIRCUMFERENCEOFWHEEL/ENCODERTICKS) * GEARRATIO;

    HardwareMap hwMap = null;

    Orientation angles;
    BNO055IMU imu;

    Telemetry telemetry;

    public void init(HardwareMap ahwMap, Telemetry tele) { //pass in hardwaremap and telemetry in the code to init stuff
        hwMap = ahwMap;
        telemetry = tele;


        fl = hwMap.dcMotor.get("front_left_motor");
        fr = hwMap.dcMotor.get("back_right_motor");
        bl = hwMap.dcMotor.get("back_left_motor");
        br = hwMap.dcMotor.get("front_right_motor");
        lift1 = hwMap.dcMotor.get("liftMotor1");
        lift2 = hwMap.dcMotor.get("liftMotor2");
        //fl.setDirection(DcMotorSimple.Direction.REVERSE);
        //bl.setDirection(DcMotorSimple.Direction.REVERSE);
        //fr.setDirection(DcMotorSimple.Direction.REVERSE);
        //br.setDirection(DcMotorSimple.Direction.REVERSE);

        clawServo = hwMap.servo.get("clawServo");
        //liftServo = hwMap.crservo.get("liftServo");

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*imu = hwMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);*/

    }

    public int tickToMM(double mm){
        return (int) (mm/TICKSTOMMTRAVEL);
    }
    public void driveForwardDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(distance);
        fr.setTargetPosition(distance);
        bl.setTargetPosition(distance);
        br.setTargetPosition(distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy()) {
        }

        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void driveBackDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(-distance);
        fr.setTargetPosition(-distance);
        bl.setTargetPosition(-distance);
        br.setTargetPosition(-distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveBack(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
        {

        }
        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveRightDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(-distance);
        fr.setTargetPosition(-distance);
        bl.setTargetPosition(distance);
        br.setTargetPosition(distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveRight(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
        {

        }
        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveLeftDistance(double power, int distance){
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setTargetPosition(distance);
        fr.setTargetPosition(distance);
        bl.setTargetPosition(-distance);
        br.setTargetPosition(-distance);

        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveLeft(power);

        while(fl.isBusy() && fr.isBusy() && bl.isBusy() && br.isBusy())
        {

        }
        stopDriving();
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void driveForward(double power) {
        fl.setPower(-power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(-power);
    }
    public void driveBack(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }
    public void driveRight(double power){
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }
    public void driveLeft(double power){
        fl.setPower(-power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(-power);
    }
    public void stopDriving() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void rotate(double wantedAngle){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = 0;
        target = wantedAngle;


        double angle = 0;

        int totalTime = 0;

        double error = 90, P, I, D, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;
        double kp = .017;
        double ki =  0;
        double kd = .02;
        while(Math.abs(error) > 1){
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            angle = angles.firstAngle;

            t = (double)System.nanoTime()/10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - angle;
            integral = ki * ((error - lastError) * dt);
            derivative = kd * ((error - lastError) / dt);

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;

            correction = P + I + D;

            fl.setPower(correction);
            fr.setPower(-correction);
            bl.setPower(correction);
            br.setPower(-correction);

            //System.out.println(P + " " + I + " " + D);
            System.out.println(error);
            System.out.println(angles);

            telemetry.addData("Dt", dt);
            telemetry.addData("Error", error);
            telemetry.addData("correction:", correction);
            telemetry.addData("top Left Power", fl.getPower());
            telemetry.addData("top Right Power", fr.getPower());
            telemetry.addData("bottom Left Power", bl.getPower());
            telemetry.addData("bottom Right Power", br.getPower());
            telemetry.addData("Angle", angle);
            telemetry.update();

            lastError = error;
            lastTime = t;
            totalTime += t;
        }
        telemetry.addData("Angle", angle);
        telemetry.addData("totalTime: ", totalTime * 10E-9);
        telemetry.update();

        System.out.println("angle: " + angle);
        System.out.println("totalTime: " + (totalTime * 10E-9));

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void lift(double power, int distance){
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(distance);
        lift2.setTargetPosition(-distance);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(power);
        lift2.setPower(power);

        while(lift1.isBusy() && lift2.isBusy()){
            telemetry.addData("enc Tickes lift1", lift1.getCurrentPosition());
            telemetry.addData("enc ticks lift2", lift2.getCurrentPosition());
            telemetry.update();
        }
        lift1.setPower(0);
        lift2.setPower(0);

        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void liftPower(double power){
        lift1.setPower(power);
        lift2.setPower(-power);
    }
    public void openClaw(){
        clawServo.setPosition(.1);
    }

    public void closeClaw(){
        clawServo.setPosition(.3);
    }
}