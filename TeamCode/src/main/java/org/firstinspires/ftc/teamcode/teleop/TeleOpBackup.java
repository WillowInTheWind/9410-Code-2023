package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "Qualifier TeleOp Backup 9410")
@Disabled
public class TeleOpBackup extends LinearOpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor slide;

    Servo leftClaw;
    Servo rightClaw;

    double slide_speed_mod = 1;
    double drive_speed_mod = .6;
    int slideStage = 0; //0 is ground, 1 is low junction, 2 is middle junction, 3 WILL BE high junction
    // State used for updating telemetry

    boolean enableSlideStages = false;

    IMU imu;

    YawPitchRollAngles robotOrientation;

    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get (DcMotor.class, "front left");
        rightFront = hardwareMap.get (DcMotor.class, "front right");
        leftBack = hardwareMap.get (DcMotor.class, "back left");
        rightBack = hardwareMap.get (DcMotor.class, "back right");
        slide = hardwareMap.get (DcMotor.class, "slide");

        leftClaw = hardwareMap.get(Servo.class, "left claw");
        rightClaw = hardwareMap.get(Servo.class, "right claw");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        //  slide.setDirection(DcMotorSimple.Direction.REVERSE);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);


        //slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slide.setTargetPosition(0);

        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(myIMUparameters);

        // Set up our telemetry dashboard


        waitForStart();

        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while (opModeIsActive()) {

            robotOrientation = imu.getRobotYawPitchRollAngles();
            // double ly = gamepad1.left_stick_y;
            // double ry = gamepad1.right_stick_y;
            // leftFront.setPower(ly);
            // leftBack.setPower(ly);
            // rightFront.setPower(ry);
            // rightBack.setPower(ry);
            double LeftStickAngle = 0;
            try {
                LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4-robotOrientation.getYaw(AngleUnit.RADIANS);
            } catch (Exception e) {
                LeftStickAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4-robotOrientation.getYaw(AngleUnit.RADIANS);
            }
            if (gamepad1.x) {
                imu.resetYaw();
            }
            if (gamepad2.y) {
                slide_speed_mod = 1;
            }
            if (gamepad2.a){
                slide_speed_mod = .5;
            }
            //drive speed control
            if (gamepad1.left_trigger > .5) {
                drive_speed_mod = .6;
            }
            if (gamepad1.left_bumper) {
                drive_speed_mod = .7;
            }
            if (gamepad1.right_bumper) {
                drive_speed_mod = 1;
            }
            if (gamepad1.right_trigger > .5) {
                leftFront.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
                leftBack.setPower(0);
            }
            //Drive speed customization
            //if (drive_speed_mod <= 2) {
            //if (gamepad1.dpad_up){
            //drive_speed_mod += .1;
            //if (gamepad1.dpad_down) {
            //drive_speed_mod -= .1;

            double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y)*drive_speed_mod;
            //double LeftStickAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4-Math.toRadians(angles.firstAngle);
            leftFront.setPower(speed*Math.cos(LeftStickAngle) + gamepad1.right_stick_x);
            leftBack.setPower(speed*Math.sin(LeftStickAngle) + gamepad1.right_stick_x);
            rightFront.setPower(speed*Math.sin(LeftStickAngle) - gamepad1.right_stick_x);
            rightBack.setPower(speed*Math.cos(LeftStickAngle) - gamepad1.right_stick_x);


            double slidePos = slide.getCurrentPosition();

            //if(Math.abs(gamepad2.left_stick_y > .4)) {
            //  enableSlideSta
            //}


            if(gamepad2.b) {
                slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                enableSlideStages = false;
            } else if(gamepad2.x) {
                slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                enableSlideStages = true;
            }
            /*while (driveClawClose){
                if (speed > 1);
                    leftClaw.setPosition(.1);
                    rightClaw.setPosition(.65);
            }*/
            if (enableSlideStages){
                if(gamepad2.dpad_down){
                    slide.setTargetPosition(1300);
                }
                else if (gamepad2.dpad_left) {
                    slide.setTargetPosition(2150);
                }
                else if (gamepad2.dpad_up) {
                    slide.setTargetPosition(2900);

                }
                if(gamepad2.dpad_right) {
                    slide.setTargetPosition(0);

                }
                if(gamepad2.left_bumper) {
                    slide.setTargetPosition(140);

                }

                slide.setPower(slide_speed_mod);

            } else {
                slide.setPower(Math.pow(gamepad2.left_stick_y,3)*drive_speed_mod);
            }


            if(gamepad2.left_trigger > .5){
                //close
                leftClaw.setPosition(.2);
                rightClaw.setPosition(.45);

            }

            if(gamepad2.right_trigger > .5){
                //open
                leftClaw.setPosition(.35);
                rightClaw.setPosition(.3);

            }

            telemetry.addData("target pos", slide.getTargetPosition());
            telemetry.addData("Slide power", slide.getPower());
            telemetry.addData("slide mode", slide.getMode());
            telemetry.addData("enableSlideStages", enableSlideStages);
            telemetry.addData("back left encoder", leftBack.getCurrentPosition());
            telemetry.addData("slide pos", slidePos);
            telemetry.update();





        }
    }

}

