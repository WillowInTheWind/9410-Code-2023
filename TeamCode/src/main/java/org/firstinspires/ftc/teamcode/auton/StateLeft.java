/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auton;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.AprilTagDetectionPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "State Left Auton")
@Disabled
public class StateLeft extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int left = 1;
    int middle = 2;
    int right = 3;
    AprilTagDetection tagOfInterest = null;

    private DcMotor backright;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor frontleft;
    private DcMotor slide;

    private Servo leftClaw;
    private Servo rightClaw;

    IMU imu;
    YawPitchRollAngles robotOrientation;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        backright = hardwareMap.get(DcMotor.class, "back right");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
        slide = hardwareMap.get (DcMotor.class, "slide");

        leftClaw = hardwareMap.get(Servo.class, "left claw");
        rightClaw = hardwareMap.get(Servo.class, "right claw");

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
//        slide.setDirection(DcMotorSimple.Direction.REVERSE)

        slide.setMode(STOP_AND_RESET_ENCODER);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        telemetry.log().add("This was built");

        closeClaw();

        //while (!gamepad1.right_bumper&& !gamepad1.left_bumper && opModeIsActive()) {
            //if (gamepad1.left_bumper){
                //utonDirection = LEFT;
                //telemetry.log().add("Auton Mode: Left");

            //if (gamepad1.right_bumper) {
               // AutonDirection = RIGHT;
                // telemetry.log().add("Auton Mode: Right");


        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == left || tag.id == middle|| tag.id == right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.addData("Slide Position", slide.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        //Raise up to avoid cone interference
        moveSlide(100);
        Drive(50,.5,Math.toRadians(90));
        sleep(250);
        sleep(250);
        //align bot
        Drive(1300, .5, Math.toRadians(0));
        sleep(250);
        //drive forward
        moveSlide(2800, .4, false);
        Drive(2075,.5,Math.toRadians(90));
        sleep(250);
        //Drive(600,.4,Math.toRadians(270));
        sleep(250);
        //Strafe Right to middle junction
        Drive(600, .5, Math.toRadians(180));
        sleep(250);
        //Raise slide to high junction
        stopRobot();
        sleep(500);
        //approach middle junction
        Drive(150,.5,Math.toRadians(90));
        moveSlide(2500, .4);
        sleep(250);
        openClaw();
        sleep(250);
        moveSlide(2800, .4);
        stopRobot();
        sleep(750);
        //drop cone
        sleep(500);
        //backup
        Drive(150,.4,Math.toRadians(270));
        moveSlide(500, .4,false);
        closeClaw();
        //lower slide
        sleep(500);
        //go to parking spot
        if(tagOfInterest == null ||tagOfInterest.id == middle) {
            //trajectory
            Drive(600,.4,Math.toRadians(180));
            sleep(1000);
        }
        else if(tagOfInterest.id == left) {
            //trajectory
            Drive(600,.4,Math.toRadians(0));
            sleep(1000);
        }

        else if(tagOfInterest.id == right) {
            //trajectory
            Drive(1800, .4, Math.toRadians(180));
            sleep(1000);
        }
    }
    private void Drive(double position, double power, double angle) {

        backleft.setMode(STOP_AND_RESET_ENCODER);
        backright.setMode(STOP_AND_RESET_ENCODER);
        frontleft.setMode(STOP_AND_RESET_ENCODER);
        frontright.setMode(STOP_AND_RESET_ENCODER);


        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DriveAngle(angle, power);
        while (Math.abs(backleft.getCurrentPosition())<Math.abs(position) && opModeIsActive()) {
            robotOrientation = imu.getRobotYawPitchRollAngles();
            DriveAngle(angle-robotOrientation.getYaw(AngleUnit.RADIANS), power);
            telemetry.addData("angle", robotOrientation.getYaw(AngleUnit.RADIANS));
            telemetry.addData("Target Position", position);
            telemetry.addData("Motor Position", backleft.getCurrentPosition());
            telemetry.update();
        }
        stopRobot();
    }
    public void turn (double power) {
        backleft.setPower(-power);
        backright.setPower(power);
        frontleft.setPower(-power);
        frontright.setPower(power);
    }
    public void turntoangle (double power, double angle) {
        turntoangle(power, angle, 10);
    }
    public void turntoangle (double power, double angle, double difference) {
        double currentAngle = getBotAngle();
        while (Math.abs(currentAngle - angle) < difference) {
            currentAngle = getBotAngle();
        }
    }
    public double getBotAngle () {
        robotOrientation = imu.getRobotYawPitchRollAngles();
        return robotOrientation.getYaw(AngleUnit.RADIANS);
    }
    public void stopRobot(){
        backleft.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        frontright.setPower(0);


    }
    private void DriveAngle(double angle, double speed) {
        angle = angle - Math.toRadians(45);
        frontleft.setPower(speed * Math.cos(angle));
        backright.setPower(speed * Math.cos(angle));
        backleft.setPower(speed * Math.sin(angle));
        frontright.setPower(speed * Math.sin(angle));
    }
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void closeClaw () {
        leftClaw.setPosition(.25);
        rightClaw.setPosition(.45);
    }
    void openClaw () {
        leftClaw.setPosition(.35);
        rightClaw.setPosition(.3);
    }
    void moveSlide(int position) {
        moveSlide(position, 1, true);
    }
    void moveSlide(int position,double power) {
        moveSlide(position, power, true);
    }
    void moveSlide (int position,double power, boolean waitForCompletion) {
        slide.setTargetPosition(position);
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slide.setPower(power);
        if (waitForCompletion) {
            while (slide.isBusy()) {
                telemetry.addData("SlidePosition", slide.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}