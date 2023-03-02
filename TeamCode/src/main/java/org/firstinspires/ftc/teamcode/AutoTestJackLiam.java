package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutoTestJackLiam", group = "OpenCV Autos" )

public class AutoTestJackLiam extends OpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor middleslideDrive = null;
    public DcMotor frontleftDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public Servo rightgripperDrive = null;
    public Servo leftgripperDrive = null;


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
    double tagSize = 0.166;
    int ID_TAG_OF_INTEREST1 = 1; // Tag ID 18 from the 36h11 family
    int ID_TAG_OF_INTEREST2 = 2;
    int ID_TAG_OF_INTEREST3 = 3;

    int tagPosition = 0;
    public IMU imu;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {

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

    public void setServo(double position, int sleep) {
        position = position * 1;
        if (position == 1) {
            while (leftgripperDrive.getPosition() != .505) {
                leftgripperDrive.setPosition(.505);
                rightgripperDrive.setPosition(.35);
            }

        }
        if (position == 0) {
            while (leftgripperDrive.getPosition() != .77) {
                leftgripperDrive.setPosition(.77);
                rightgripperDrive.setPosition(.12);
            }

        }
        sleep(sleep);

    }

    public void setSliderUp(double speed, double level,int sleep) {
        middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");
        boolean finished = false;
        double position = 0;
        if (level == 0) {position = 0;}
        if (level == 1) {position = 200;}
        if (level == 2) {position = 500;}
        if (level == 3) {position = 700;}
        if (level == 4) {position = 1000;}
        if (level == 5) {position = 1700;}
        if (level == 6) {position = 2600;}
        if (level == 7) {position = 4300;}


        if (middleslideDrive.getCurrentPosition() < position) {
            while (middleslideDrive.getCurrentPosition()< position) {
                middleslideDrive.setPower(-speed);
            }
            telemetry();
        }
        middleslideDrive.setPower(0);
        sleep(sleep);
    }
    public void setSliderDown(double speed, double level,int sleep) {
        middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");
        boolean finished = false;
        double position = 0;
        if (level == 0) {position = 0;}
        if (level == 1) {position = 200;}
        if (level == 2) {position = 500;}
        if (level == 3) {position = 700;}
        if (level == 4) {position = 1000;}
        if (level == 5) {position = 1700;}
        if (level == 6) {position = 2600;}
        if (level == 7) {position = 4300;}


        if (middleslideDrive.getCurrentPosition() > position) {
            while (middleslideDrive.getCurrentPosition()> position) {
                middleslideDrive.setPower(speed);
            }
            telemetry();
        }
        middleslideDrive.setPower(0);
        sleep(sleep);
    }

    public void placeCone() {
        setSliderDown(.5,0,500);
        moveSimpleEncoder(.5,100,1,100);
        setServo(0,500);
        setSliderDown(.5,0,500);
        moveSimpleEncoder(.5,100,3,100);

    }
    public void moveSimpleEncoder(double speed, int distance, int direction, int sleep) {

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (direction == 1) {
            frontleftDrive.setTargetPosition(distance);
            frontrightDrive.setTargetPosition(distance);
            backleftDrive.setTargetPosition(distance);
            backrightDrive.setTargetPosition(distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(speed);
        }
        if (direction == 2) {
            frontleftDrive.setTargetPosition(-distance);
            frontrightDrive.setTargetPosition(distance);
            backleftDrive.setTargetPosition(distance);
            backrightDrive.setTargetPosition(-distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(-speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(-speed);
        }
        if (direction == 3) {
            frontleftDrive.setTargetPosition(-distance);
            frontrightDrive.setTargetPosition(-distance);
            backleftDrive.setTargetPosition(-distance);
            backrightDrive.setTargetPosition(-distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(-speed);
            frontrightDrive.setPower(-speed);
            backleftDrive.setPower(-speed);
            backrightDrive.setPower(-speed);
        }
        if (direction == 4) {
            frontleftDrive.setTargetPosition(distance);
            frontrightDrive.setTargetPosition(-distance);
            backleftDrive.setTargetPosition(-distance);
            backrightDrive.setTargetPosition(distance);

            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(-speed);
            backleftDrive.setPower(-speed);
            backrightDrive.setPower(speed);
        }


        while (frontleftDrive.isBusy() && frontrightDrive.isBusy() && backleftDrive.isBusy() && backrightDrive.isBusy()) {
            telemetry();
            // Wait for the motors to reach their target positions and updates telemetry
        }

        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);

        sleep(sleep);
    }



    public void telemetry() {
        telemetry.addData("Run Time", runtime.toString());
        telemetry.addData("Front Right Encoder", frontrightDrive.getCurrentPosition());
        telemetry.addData("Front Left Encoder", frontleftDrive.getCurrentPosition());
        telemetry.addData("Back Right Encoder", backrightDrive.getCurrentPosition());
        telemetry.addData("Back Left Encoder", backleftDrive.getCurrentPosition());
        telemetry.addData("IMU Z Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
    }

}