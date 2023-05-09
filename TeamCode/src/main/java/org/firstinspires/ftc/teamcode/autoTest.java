package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.apache.commons.math3.analysis.function.Max;
import org.apache.commons.math3.util.FastMath;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "autoTest", group = "OpenCV Autos" )

public class autoTest extends LinearOpMode {

    static int slideEpislon = 200;
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor middleSlideDrive = null;
    private boolean hasRun = false;
    public DcMotor frontleftDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public Servo gripperDrive = null;
    public ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;

    SampleMecanumDrive drive;
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

    int tagPosition = 0;

    int tagOfInterest1 = 1; // Tag ID 1 from the 36h11 family
    int tagOfInterest2 = 2; // Tag ID 2 from the 36h11 family
    int tagOfInterest3 = 3; // Tag ID 3 from the 36h11 family
    public IMU imu;

    //State
    public enum AutoRunState
    {
        STATE_BEGIN,
        STATE_FIND_CONE,
        STATE_ROTATE,
        STATE_TO_CONE,
        STATE_WAIT_FOR_LIFT,
        STATE_DROP_CONE,
        STATE_FINISH
    };
    AutoRunState currentState=AutoRunState.STATE_BEGIN;
    AutoRunState nextState=AutoRunState.STATE_BEGIN;

    public enum ClawState
    {
        STATE_CLAW_IDLE,
        STATE_CLAW_CLOSE,
        STATE_CLAW_MOVE,
        STATE_CLAW_OPEN,
        STATE_CLAW_CLOSE_AND_MOVE
    };
    ClawState currentClawState=ClawState.STATE_CLAW_IDLE;
    int goalSlidePosition = 0;
    double rotationalGoal = 0;
    AprilTagDetection tagOfInterest = null;
    private static ElapsedTime stopwatch = new ElapsedTime();
    @Override
    public void runOpMode() {

        initialize();

        drive = new SampleMecanumDrive(hardwareMap);

        /*Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90));

        drive.setPoseEstimate(startPose);
        TrajectorySequence Tag1Ending = drive.trajectorySequenceBuilder(startPose)
                .addDisplacementMarker(() -> {
                    setServo(1,0);
                })
                .lineTo(new Vector2d(-36,-38))
                .strafeLeft(24)
                .build();

        drive.followTrajectorySequence(Tag1Ending);
*/
        Pose2d startPose = new Pose2d(-60, -36, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        TrajectorySequence generalMovement = drive.trajectorySequenceBuilder(startPose) //Lines Up To Pole
                .lineTo(new Vector2d(-65, -36 ))

                .lineTo(new Vector2d(-60, 50 ))
                        .build();
               /* .lineTo(new Vector2d(60, -60))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(60, 60))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-48, 60))
                .build(); */
        drive.followTrajectorySequenceAsync(generalMovement);

        telemetry.addData("Status", "BEGIN");
        telemetry.update();
        while (true) {
            mainLoop();
            updateClaw();
        }

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

    public void initialize()
    {
        // Motors and Servos
        frontleftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontrightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backleftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backrightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        middleSlideDrive = hardwareMap.get(DcMotor.class, "middle_slides_drive");

        gripperDrive = hardwareMap.get(Servo.class, "gripper_drive");
        //sensors
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleSlideDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleSlideDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopwatch.reset();
        //Set up IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        // Wait for the game to start (driver presses PLAY)
        imu.initialize(myIMUparameters);
        imu.resetYaw();
        SetSlidePosition(400);

        telemetry.setMsTransmissionInterval(50);
    }

    public void mainLoop() {
        //telemetry.addData("ColorTest:", colorTest() ? "Win" : "Fail");
        telemetry.addData("CurrentState", currentState);
        telemetry.addData("time", stopwatch.time());
        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();

        telemetry.addData("imu yaw:", ypr.getYaw(AngleUnit.DEGREES));
        //telemetry.addData("imu pitch:", ypr.getPitch(AngleUnit.DEGREES));
        //telemetry.addData("imu roll:", ypr.getRoll(AngleUnit.DEGREES));
        //telemetry.addData("imu Orientation:", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

        switch (currentState) {
            case STATE_BEGIN: {
                if (drive.isBusy()) {
                    drive.update();
                } else {
                    currentClawState = ClawState.STATE_CLAW_OPEN;
                    currentState = AutoRunState.STATE_FIND_CONE;
                }
                break;
            }
            case STATE_FIND_CONE: {
                if (distanceSensor.getDistance(DistanceUnit.CM) < 50) {
                    frontleftDrive.setPower(-1);
                    frontrightDrive.setPower(-1);
                    backleftDrive.setPower(-1);
                    backrightDrive.setPower(-1);
                    RequestRotation(90, AutoRunState.STATE_TO_CONE);
                } else {
                    double searchPower = 0.4;
                    frontleftDrive.setPower(searchPower);
                    frontrightDrive.setPower(searchPower);
                    backleftDrive.setPower(searchPower);
                    backrightDrive.setPower(searchPower);
                }
                break;
            }
            case STATE_ROTATE: {
                double yaw = ypr.getYaw(AngleUnit.DEGREES);
                double turnError = (1.0-Math.abs(yaw/rotationalGoal));
                double turnPower = Math.copySign(FastMath.min(1.0, turnError)*0.9, rotationalGoal);
                turnPower = Math.copySign(FastMath.max(0.05, Math.abs(turnPower)), turnPower);
                turnPower = Math.copySign(1.0, turnError)*turnPower;
                telemetry.addData("Turn Power:", turnPower);
                if (Math.abs(turnError)<0.02)
                {
                    currentState = nextState;
                    stopwatch.reset();
                }
                else
                {
                    frontleftDrive.setPower(-turnPower);
                    frontrightDrive.setPower(turnPower);
                    backleftDrive.setPower(-turnPower);
                    backrightDrive.setPower(turnPower);
                }
                break;
            }
            case STATE_TO_CONE: {
                if (stopwatch.time() > 0.55) {
                    frontleftDrive.setPower(0);
                    frontrightDrive.setPower(0);
                    backleftDrive.setPower(0);
                    backrightDrive.setPower(0);
                    SetSlidePosition(1200);
                    currentClawState = ClawState.STATE_CLAW_CLOSE_AND_MOVE;
                    currentState = AutoRunState.STATE_WAIT_FOR_LIFT;
                } else {
                    double searchPower = 0.4;
                    frontleftDrive.setPower(searchPower);
                    frontrightDrive.setPower(searchPower);
                    backleftDrive.setPower(searchPower);
                    backrightDrive.setPower(searchPower);
                }
                break;
            }
            case STATE_WAIT_FOR_LIFT: {
                if (currentClawState == ClawState.STATE_CLAW_IDLE) {

                    RequestRotation(-45, AutoRunState.STATE_DROP_CONE);
                }
                break;
            }
            case STATE_DROP_CONE: {
                currentClawState = ClawState.STATE_CLAW_OPEN;
                RequestRotation(-45, AutoRunState.STATE_FINISH);
            }
            case STATE_FINISH: {
                frontleftDrive.setPower(0);
                frontrightDrive.setPower(0);
                backleftDrive.setPower(0);
                backrightDrive.setPower(0);
                //SetSlidePosition(0);
                break;
            }
        }
        telemetry.update();
    }

    public void updateClaw() {
        telemetry.addData("CurrentClawState", currentClawState);
        telemetry.addData("Gripper Position", gripperDrive.getPosition());
        telemetry.addData("SlidePosition", middleSlideDrive.getCurrentPosition());

        switch (currentClawState) {
            case STATE_CLAW_OPEN: {
                gripperDrive.setPosition(0.25);
                currentClawState = ClawState.STATE_CLAW_IDLE;
                break;
            }
            case STATE_CLAW_CLOSE: {
                gripperDrive.setPosition(0.0);
                currentClawState = ClawState.STATE_CLAW_IDLE;
                break;
            }
            case STATE_CLAW_MOVE: {
                telemetry.addData("Slide GOAL", goalSlidePosition);
                if (Math.abs(middleSlideDrive.getCurrentPosition()-goalSlidePosition) < slideEpislon)
                {
                    currentClawState = ClawState.STATE_CLAW_IDLE;
                    middleSlideDrive.setPower(0);
                }
                else if (middleSlideDrive.getCurrentPosition() <= goalSlidePosition) {
                    middleSlideDrive.setPower(0.2);
                }
                else {
                    middleSlideDrive.setPower(-0.2);
                }
                break;
            }
            case STATE_CLAW_CLOSE_AND_MOVE: {
                telemetry.addData("Slide GOAL", goalSlidePosition);
                gripperDrive.setPosition(0.0);
                if (gripperDrive.getPosition() == 0.0) {
                    if (Math.abs(middleSlideDrive.getCurrentPosition()-goalSlidePosition) < slideEpislon)
                    {
                        currentClawState = ClawState.STATE_CLAW_IDLE;
                        middleSlideDrive.setPower(0);
                    }
                    else if (middleSlideDrive.getCurrentPosition() <= goalSlidePosition) {
                        middleSlideDrive.setPower(0.2);
                    }
                    else {
                        middleSlideDrive.setPower(-0.2);
                    }
                }

                break;
            }
            case STATE_CLAW_IDLE: {
                break;
            }
        }
    }

    public void SetSlidePosition(int position)
    {
        if (position!=goalSlidePosition) {
            goalSlidePosition = position;
            currentClawState = ClawState.STATE_CLAW_MOVE;
        }
    }
    public void RequestRotation(double angle, AutoRunState stateNext)
    {
       // YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        imu.resetYaw();
        nextState = stateNext;
        rotationalGoal = angle;// - ypr.getYaw(AngleUnit.DEGREES);
        currentState = AutoRunState.STATE_ROTATE;
        if (rotationalGoal == 0) {
            rotationalGoal = 1;
        }
    }

    /*public boolean colorTest ()
    {
        telemetry.addData("colorSensor alpha", colorSensor.alpha());
        telemetry.addData("colorSensor blue", colorSensor.blue());

        if (colorSensor.alpha() > 200)
        {
            return true;
        }
        return false;
    }*/
}