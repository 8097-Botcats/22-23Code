package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
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


//TELEOP WITH TWO CONTROLLERS GP1 IS DRIVE GP2 IS OTHER
@TeleOp(name = "Tele w/ 2")
public class teleopOff2 extends LinearOpMode {
    public void runOpMode() {

        DcMotor fl = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor fr = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
        DcMotor lift1 = hardwareMap.dcMotor.get("liftMotor1");
        DcMotor lift2 = hardwareMap.dcMotor.get("liftMotor2");
        //fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        //bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo clawServo = hardwareMap.servo.get("clawServo");

        double driveSpeed = 0.4;
        final int liftHome = 0;
        boolean fieldCentric = false;
        double finalAngle;
        double robotAngle;
        boolean speedToggle = true;
        boolean yButton = true;
        boolean xButton = true;
        boolean bButton = true;

        double liftEncoder = 0;

        Orientation angles;
        BNO055IMU imu;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        telemetry.addData("Status", "Initalized");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            //speed control gamepad1

            if (gamepad1.right_trigger > 0 ) {
                driveSpeed = 0.7;
            }
            else {
                driveSpeed = 0.4;
            }

            if (gamepad1.y && yButton){
                yButton = false;
                fieldCentric = !fieldCentric;
            }
            if (!gamepad1.y && !yButton) {
                yButton = true;
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            robotAngle = angles.firstAngle;

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y); //finds hypotenuse (power of each motor)
            double gpAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4; //finds angle of robot subtracted by pi/4 bc
            //it "shifts" the powers to each motor CW
            double rightX = (-gamepad1.right_stick_x) * .8; //for rotating w/ right stick

            if(fieldCentric){
                finalAngle = gpAngle - robotAngle;
            }
            else{
                finalAngle = gpAngle;
            }

            final double v1 = driveSpeed * (r * Math.cos(finalAngle) + rightX);
            final double v2 = driveSpeed * (r * Math.sin(finalAngle) - rightX);
            final double v3 = driveSpeed * (r * Math.sin(finalAngle) + rightX);
            final double v4 = driveSpeed * (r * Math.cos(finalAngle) - rightX);
            /*
                r is multiplier for power
                math.cos is used for fl and br bc fl&br are used to go diagonal top right, if you want to go faster to the right apply more power to
                those motors so closer joystick is to x axis faster robot go to that direction
                math.sin is used for same reason as ^ but to go faster forward/backwards
             */

            fl.setPower(v1);
            fr.setPower(v2);
            bl.setPower(v3);
            br.setPower(v4);

            if((gamepad2.right_trigger > 0) && gamepad2.left_trigger == 0){
                lift1.setPower(gamepad2.right_trigger);
                lift2.setPower(-gamepad2.right_trigger);
            }
            if((gamepad2.left_trigger > 0) && gamepad2.right_trigger == 0){
                lift1.setPower(-gamepad2.left_trigger);
                lift2.setPower(gamepad2.left_trigger);
            }
            if(gamepad2.right_trigger == 0 && gamepad2.left_trigger == 0){
                lift1.setPower(0);
                lift2.setPower(0);
            }


            if (gamepad2.x && xButton) {
                telemetry.addData("flag 1", "entered gamepad1x&&xbutton loop");
                telemetry.update();
                xButton = false;
                clawServo.setPosition(.3);
            }
            if (!gamepad2.x && !xButton) {
                telemetry.addData("flag 2", "entered !gamepad1x&&!xbutton loop");
                telemetry.update();
                xButton = true;
            }

            if (gamepad2.b && bButton) {
                telemetry.addData("flag 3", "entered gamepad1b&&bbutton loop");
                telemetry.update();
                bButton = false;
                clawServo.setPosition(.1);
            }
            if(!gamepad2.b && !bButton){
                telemetry.addData("flag 4", "entered !gamepad1b&&b!button loop");
                telemetry.update();
                bButton = true;
            }

            telemetry.addData("Drive Speed", driveSpeed);
            telemetry.addData("Field Centric", fieldCentric);
            telemetry.addData("lift1 enc ticks", lift1.getCurrentPosition());
            telemetry.addData("lift2 enc ticks", lift2.getCurrentPosition());
            telemetry.update();
        }
    }
}
