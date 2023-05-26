package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "auto4", group = "tournament Autos" )

public class auto4 extends LinearOpMode {
    final double gripperChangeTime = 1;
    final int slideEpislon = 200;
    final double floatEpislon = 0.03;
    final double rotationalEpsilon = 0.04;
    static double dancePower = -0.4;
    static int danceRepeats = 0;
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor middleSlideDrive = null;
    private boolean hasRun = false;
    public DcMotor frontleftDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public Servo gripperDrive = null;
    public NormalizedColorSensor colorSensor = null;
    private DistanceSensor distanceSensorTwo = null;
    private DistanceSensor distanceSensor = null;


    //Sound resources
    int silverSoundID = 0;
    int goldSoundID = 0;
    int crow1SoundID = 0;
    int crow2SoundID = 0;
    int crow3SoundID = 0;
    int chachaslideSoundID = 0;
    //State progress markers
    boolean SubStateInitialized = false;
    boolean SubStateStarted = false;

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
        STATE_DANCE,
        STATE_DROP_CONE,
        STATE_DRIVE_TO_COLOR,
        STATE_MOVE,
        STATE_FINISH
    };
    AutoRunState currentState=AutoRunState.STATE_BEGIN;
    AutoRunState nextState=AutoRunState.STATE_BEGIN;

    public enum ClawState
    {
        STATE_CLAW_IDLE,
        STATE_CLAW_CLOSE,
        STATE_CLAW_OPEN,
    };
    ClawState currentClawState=ClawState.STATE_CLAW_IDLE;

    public enum SlideState
    {
        STATE_SLIDE_IDLE,
        STATE_SLIDE_MOVE
    }
    SlideState currentSlideState=SlideState.STATE_SLIDE_IDLE;

    int goalSlidePosition = 0;
    double currentRotationalGoal = 0;
    double prevRotationalGoal = 0;
    double totalRotationCorrection=0;

    AprilTagDetection tagOfInterest = null;
    private static ElapsedTime stopwatch = new ElapsedTime();
    private static ElapsedTime gripperwatch = new ElapsedTime();
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

                .lineTo(new Vector2d(-60, 40 ))
                .build();
               /* .lineTo(new Vector2d(60, -60))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(60, 60))
                .turn(Math.toRadians(90))
                .lineTo(new Vector2d(-48, 60))
                .build(); */
        drive.followTrajectorySequenceAsync(generalMovement);

        waitForStart();

        telemetry.addData("Status", "BEGIN");
        telemetry.update();
        currentState=AutoRunState.STATE_BEGIN;
        while (currentState != AutoRunState.STATE_FINISH) {
            mainLoop();
            updateClaw();
            updateSlide();
            updateCamera();

        }

    }
    public void telemetry() {
        //telemetry.addData("ColorTest:", colorTest() ? "Win" : "Fail");
        telemetry.addData("CurrentState", currentState);
        telemetry.addData("time", stopwatch.time());
        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("distance2", distanceSensorTwo.getDistance(DistanceUnit.CM));
        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();

        telemetry.addData("imu yaw:", ypr.getYaw(AngleUnit.DEGREES));
        //telemetry.addData("imu pitch:", ypr.getPitch(AngleUnit.DEGREES));
        //telemetry.addData("imu roll:", ypr.getRoll(AngleUnit.DEGREES));
        //telemetry.addData("imu Orientation:", imu.getRobotOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);

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
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        colorSensor.setGain(15.0f);
        distanceSensorTwo = hardwareMap.get(DistanceSensor.class, "distanceSensorTwo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");


        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        backleftDrive.setDirection(DcMotor.Direction.REVERSE);
        backrightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleSlideDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        // Sound resources (see SoundTest.java)
        silverSoundID = hardwareMap.appContext.getResources().getIdentifier("silver", "raw", hardwareMap.appContext.getPackageName());
        goldSoundID   = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());
        crow1SoundID   = hardwareMap.appContext.getResources().getIdentifier("crow1",   "raw", hardwareMap.appContext.getPackageName());
        crow2SoundID   = hardwareMap.appContext.getResources().getIdentifier("crow2",   "raw", hardwareMap.appContext.getPackageName());
        crow3SoundID   = hardwareMap.appContext.getResources().getIdentifier("crow3",   "raw", hardwareMap.appContext.getPackageName());
        chachaslideSoundID   = hardwareMap.appContext.getResources().getIdentifier("chachaslide",   "raw", hardwareMap.appContext.getPackageName());
        stopwatch.reset();
        gripperwatch.reset();
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
        SetSlidePosition(500);

        telemetry.setMsTransmissionInterval(50);
    }

    public void mainLoop() {
        telemetry();
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
                   // SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                    frontleftDrive.setPower(0);
                    frontrightDrive.setPower(0);
                    backleftDrive.setPower(0);
                    backrightDrive.setPower(0);
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
                YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
                double yaw = ypr.getYaw(AngleUnit.DEGREES);
                double turnError = currentRotationalGoal-yaw;
                double turnT = turnError/totalRotationCorrection;
                double turnPower = FastMath.min(1.0, turnT)*1.0;
                turnPower = Math.copySign(FastMath.max(0.05, Math.abs(turnPower)), totalRotationCorrection);
                turnPower = Math.copySign(1.0, turnT)*turnPower;
                telemetry.addData("Turn Error:", turnError);
                telemetry.addData("Turn T:", turnT);
                telemetry.addData("Turn Power:", turnPower);
                if (Math.abs(turnError)<rotationalEpsilon)
                {
                    frontleftDrive.setPower(0);
                    frontrightDrive.setPower(0);
                    backleftDrive.setPower(0);
                    backrightDrive.setPower(0);
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
                if (distanceSensorTwo.getDistance(DistanceUnit.CM) < 10) {
                    frontleftDrive.setPower(0);
                    frontrightDrive.setPower(0);
                    backleftDrive.setPower(0);
                    backrightDrive.setPower(0);
                    currentClawState = currentClawState.STATE_CLAW_CLOSE;
                    currentState = AutoRunState.STATE_WAIT_FOR_LIFT;
                } else {
                    double searchPower = 0.2;
                    frontleftDrive.setPower(searchPower);
                    frontrightDrive.setPower(searchPower);
                    backleftDrive.setPower(searchPower);
                    backrightDrive.setPower(searchPower);
                }
                break;
            }
            case STATE_WAIT_FOR_LIFT: {
                if (currentClawState == ClawState.STATE_CLAW_IDLE) {
                    SetSlidePosition(1200);
                    /*stopwatch.reset();
                    if (stopwatch.time() < 0.2) {
                        double searchPower = 0.2;
                        frontleftDrive.setPower(-searchPower);
                        frontrightDrive.setPower(-searchPower);
                        backleftDrive.setPower(-searchPower);
                        backrightDrive.setPower(-searchPower);
                    }*/
                }
                if (currentClawState == ClawState.STATE_CLAW_IDLE && currentSlideState == SlideState.STATE_SLIDE_IDLE) {
                   currentState = AutoRunState.STATE_DANCE;

                }
                break;
            }
            case STATE_DANCE: {
                if (!SubStateInitialized)
                {
                    danceRepeats = 0;
                    SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, chachaslideSoundID);
                    SubStateInitialized = true;
                    stopwatch.reset();
                }
                if (danceRepeats == 5) {
                    currentState = AutoRunState.STATE_DROP_CONE;
                    SubStateInitialized = false;
                    double searchPower = 0.0;
                    frontleftDrive.setPower(searchPower);
                    frontrightDrive.setPower(searchPower);
                    backleftDrive.setPower(searchPower);
                    backrightDrive.setPower(searchPower);
                    break;
                }
                if (stopwatch.time() < 1.2) {
                    frontleftDrive.setPower(-dancePower);
                    frontrightDrive.setPower(dancePower);
                    backleftDrive.setPower(dancePower);
                    backrightDrive.setPower(-dancePower);
                } else {
                    stopwatch.reset();
                    dancePower = -dancePower;
                    danceRepeats++;
                }
                break;
            }
            case STATE_DROP_CONE: {
                if (!SubStateInitialized)
                {
                    SubStateInitialized = true;
                    SetSlidePosition(100);
                }
                if (currentSlideState == SlideState.STATE_SLIDE_IDLE && !SubStateStarted) {
                    currentClawState = ClawState.STATE_CLAW_OPEN;
                   // SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                    SubStateStarted = true;
                }
                if (currentSlideState == SlideState.STATE_SLIDE_IDLE && currentClawState == ClawState.STATE_CLAW_IDLE) {
                    SetSlidePosition(780);
                    if (currentSlideState == SlideState.STATE_SLIDE_IDLE && currentClawState == ClawState.STATE_CLAW_IDLE) {
                        currentState = AutoRunState.STATE_FINISH;
                        SubStateInitialized=false;
                        SubStateStarted=false;
                    }
                }
                break;
            }
            case STATE_DRIVE_TO_COLOR:
            {
                if (colorTest())
                {
                    RequestRotation(90, AutoRunState.STATE_MOVE);
                } else {
                    double searchPower = 0.4;
                    frontleftDrive.setPower(searchPower);
                    frontrightDrive.setPower(searchPower);
                    backleftDrive.setPower(searchPower);
                    backrightDrive.setPower(searchPower);
                }

                break;
            }
            case STATE_MOVE: {
                if (!SubStateInitialized)
                {
                    SubStateInitialized = true;
                    stopwatch.reset();
                }
                if (stopwatch.time() < 0.95) {
                    SetSlidePosition(-100);
                    double searchPower = 0.4;
                    frontleftDrive.setPower(searchPower);
                    frontrightDrive.setPower(searchPower);
                    backleftDrive.setPower(searchPower);
                    backrightDrive.setPower(searchPower);
                } else {
                    SubStateInitialized = false;
                    currentState = AutoRunState.STATE_FINISH;
                }
                break;
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
    public void updateCamera() {
        if (tagOfInterest == null) {
            telemetry.addLine("xaeruac upadete(");
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == tagOfInterest1) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPosition = 1;
                        break;
                    }

                    if (tag.id == tagOfInterest2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPosition = 2;
                        break;
                    }

                    if (tag.id == tagOfInterest3) {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagPosition = 3;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight at Position " + tagPosition);
                    cameraPlaySound();
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                }
            }

            //sleep(20);  // is required or else system will break
        }
    }
    public void cameraPlaySound() {
        if (tagPosition == 1) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, crow1SoundID);
        } else if (tagPosition == 2) {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, crow2SoundID);
        } else {
            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, crow3SoundID);
        }
    }
    public void updateClaw() {
        telemetry.addData("CurrentClawState", currentClawState);
        telemetry.addData("Gripper Position", gripperDrive.getPosition());
        telemetry.addData("Gripper time", gripperwatch.time());


        switch (currentClawState) {
            case STATE_CLAW_OPEN: {
                final double OpenPosition = 0.45;
                if (Math.abs(gripperDrive.getPosition() - OpenPosition)>floatEpislon) {
                    gripperDrive.setPosition(OpenPosition);
                   // SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, silverSoundID);
                    gripperwatch.reset();
                }
                if (gripperwatch.time() > gripperChangeTime)
                {
                    currentClawState = ClawState.STATE_CLAW_IDLE;
                }
                break;
            }
            case STATE_CLAW_CLOSE: {
                if (Math.abs(gripperDrive.getPosition() )>floatEpislon) {
                    gripperDrive.setPosition(0.0);
                   // SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, goldSoundID);
                    gripperwatch.reset();
                }
                if (gripperwatch.time() > gripperChangeTime) {
                    currentClawState = ClawState.STATE_CLAW_IDLE;
                }
                break;
            }
            case STATE_CLAW_IDLE: {
                break;
            }
        }
    }
    public void updateSlide() {
        telemetry.addData("CurrentSlideState", currentSlideState);
        telemetry.addData("SlidePosition", middleSlideDrive.getCurrentPosition());

        switch (currentSlideState) {

            case STATE_SLIDE_MOVE: {
                telemetry.addData("Slide GOAL", goalSlidePosition);
                if (Math.abs(middleSlideDrive.getCurrentPosition()-goalSlidePosition) < slideEpislon)
                {
                    currentSlideState = SlideState.STATE_SLIDE_IDLE;
                    middleSlideDrive.setPower(0);
                }
                else if (middleSlideDrive.getCurrentPosition() <= goalSlidePosition) {
                    middleSlideDrive.setPower(0.3);
                }
                else {
                    middleSlideDrive.setPower(-0.3);
                }
                break;
            }

            case STATE_SLIDE_IDLE: {
                break;
            }
        }
    }


    public void SetSlidePosition(int position)
    {
        if (position!=goalSlidePosition) {
            goalSlidePosition = position;
            currentSlideState = currentSlideState.STATE_SLIDE_MOVE;
        }
    }
    public void RequestRotation(double angle, AutoRunState stateNext)
    {
        // YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        //imu.resetYaw();
        //rotationalGoal += angle;// - ypr.getYaw(AngleUnit.DEGREES);
        //if (rotationalGoal == 0) {
        //    rotationalGoal = 1;
        //}

        YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
        prevRotationalGoal = currentRotationalGoal;
        currentRotationalGoal += angle;
        totalRotationCorrection = (currentRotationalGoal - ypr.getYaw(AngleUnit.DEGREES));
        if (Math.abs(totalRotationCorrection)<0.01 )
            return;
        nextState = stateNext;
        currentState = AutoRunState.STATE_ROTATE;
    }

    public boolean colorTest ()
    {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        telemetry.addData("colorSensor alpha", colors.alpha);
        telemetry.addData("colorSensor blue", colors.blue);
        telemetry.addData("colorSensor red", colors.red);
        telemetry.addData("colorDistance", ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));

        if (colors.red > 0.0125f)
        {
            return true;
        }
        return false;
        /*if (colors.blue > 0.0125f)
        {
            return true;
        }
        return false;*/
    }
}