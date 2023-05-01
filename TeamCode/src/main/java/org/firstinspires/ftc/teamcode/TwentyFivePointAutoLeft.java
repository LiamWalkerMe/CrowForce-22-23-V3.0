package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "TwentyFivePointAutoLeft", group = "OpenCV Autos" )
@Config
public class TwentyFivePointAutoLeft extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor middleslideDrive = null;
    public DcMotor frontleftDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public Servo gripperDrive = null;

    private DistanceSensor distanceSensor;

    public static double leftStrafe = -4;


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
    public void runOpMode() {
        // Motors and Servos
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        gripperDrive = hardwareMap.get(Servo.class, "gripper_drive");

        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleslideDrive.setDirection(DcMotor.Direction.FORWARD);
        middleslideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleslideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleslideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        //Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                // does nothing if error
            }
        });

        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        TrajectorySequence allMovement = drive.trajectorySequenceBuilder(startPose) //Lines Up To Pole
                .lineTo(new Vector2d(-36, -36))
                .lineTo(new Vector2d(-1, -36))
                .waitSeconds(.25)
                .addTemporalMarker(10, () -> {
                    //horizontalDetect();
                    detectPole();
                    placeCone();
                    int parkDistance = 0;
                    if (tagPosition == 3) {parkDistance = 1250;}
                    if (tagPosition == 2) {parkDistance = 4000;}
                    if (tagPosition == 1) {parkDistance = 6300;}
                    moveSimpleEncoder(.5, parkDistance,2,250);
                })
                .build();

        TrajectorySequence failedAprilTag = drive.trajectorySequenceBuilder(startPose)
                .lineTo(new Vector2d(-36, -36))
                .build();


        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST1) {
                        tagPosition = 1;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST2) {
                        tagPosition = 2;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    if (tag.id == ID_TAG_OF_INTEREST3) {
                        tagPosition = 3;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest == null) {

            drive.followTrajectorySequence(failedAprilTag);

        } else {
            /*
             * Insert your autonomous code here, probably using the tag pose to decide your configuration.
             */

            if (tagPosition == 1) {

                setServo(1,100);
                drive.followTrajectorySequence(allMovement);

            } else if (tagPosition == 2) {

                setServo(1,100);
                drive.followTrajectorySequence(allMovement);

            } else if (tagPosition == 3) {

                setServo(1,100);
                drive.followTrajectorySequence(allMovement);

            }
        }
    }
    public void placeCone() {
        setSliderUp(.5,7,500);
        moveSimpleEncoder(.25,200,1,100);
        setServo(0,500);
        setSliderDown(.5,0,500);
        moveSimpleEncoder(.25,200,3,100);
        sleep(250);
    }
    public void setServo(double position, int sleep) {
        if (position == 1) {
            gripperDrive.setPosition(0);
            sleep(sleep);
        }
        if (position == 0) {
            gripperDrive.setPosition(.25);
            sleep(sleep);
        }

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
        if (level == 7) {position = 4200;}


        if (middleslideDrive.getCurrentPosition() < position) {
            while (middleslideDrive.getCurrentPosition()< position) {
                middleslideDrive.setPower(speed);
                telemetry();
            }
            telemetry();
        }
        middleslideDrive.setPower(0);
        sleep(sleep);
    }
    public void setSliderDown(double speed, double level,int sleep) {
        middleslideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");
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
                telemetry();
                middleslideDrive.setPower(-speed);
            }
            telemetry();
        }
        middleslideDrive.setPower(0);
        sleep(sleep);
    }


    public void detectPole() {
        while (distanceSensor.getDistance(DistanceUnit.CM) > 13) {
            frontleftDrive.setPower(.1);
            frontrightDrive.setPower(.1);
            backleftDrive.setPower(.1);
            backrightDrive.setPower(.1);
        }
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(.0);
        backrightDrive.setPower(0);
        sleep(250);
    }
    public void horizontalDetect() {
        while (distanceSensor.getDistance(DistanceUnit.CM) > 15.15) {
            frontleftDrive.setPower(.15);
            frontrightDrive.setPower(-.15);
            backleftDrive.setPower(-.15);
            backrightDrive.setPower(.15);
        }
        frontleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backleftDrive.setPower(0);
        backrightDrive.setPower(0);
        /*if (distanceSensor.getDistance(DistanceUnit.CM) > 17.6 || distanceSensor.getDistance(DistanceUnit.CM) <17.4)
        {

            while (distanceSensor.getDistance(DistanceUnit.CM) > 17.6 || distanceSensor.getDistance(DistanceUnit.CM) <17.4)
            {
                double initialDistance = distanceSensor.getDistance(DistanceUnit.CM);
                double errorTime =


            }
        }
        */
        sleep(250);
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

            frontleftDrive.setPower(speed);
            frontrightDrive.setPower(-speed);
            backleftDrive.setPower(-speed);
            backrightDrive.setPower(speed);
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

            frontleftDrive.setPower(-speed);
            frontrightDrive.setPower(speed);
            backleftDrive.setPower(speed);
            backrightDrive.setPower(-speed);
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
        telemetry.addData("middleSlides Encoder", middleslideDrive.getCurrentPosition());
        telemetry.addData("middleSlides IsBusy", middleslideDrive.isBusy());
        telemetry.addData("IMU Z Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();
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


}