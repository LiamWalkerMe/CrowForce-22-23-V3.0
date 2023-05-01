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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor middleSlideDrive = null;
    private boolean hasRun = false;
    public DcMotor frontleftDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;
    public DcMotor middleslideDrive = null;
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

    AprilTagDetection tagOfInterest = null;

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
                .lineTo(new Vector2d(-30, -60))
                .turn(Math.toRadians(90))
                .build();
        /*        .lineTo(new Vector2d(-60, -60))
                .lineTo(new Vector2d(60, -60))
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
        }

    }
    public void setServo(double position, int sleep) {
        position = position * 1;
        if (position == 1) {
            while (gripperDrive.getPosition() != .505) {
                gripperDrive.setPosition(.505);
            }

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


        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        telemetry.setMsTransmissionInterval(50);
    }

    public void mainLoop()
    {
        telemetry.addData("ColorTest:", colorTest() ? "Win" : "Fail");
        if (!colorTest()) {
            drive.update();
        }
        telemetry.update();
    }

    public boolean colorTest ()
    {
        telemetry.addData("colorSensor alpha", colorSensor.alpha());
        telemetry.addData("colorSensor blue", colorSensor.blue());

        if (colorSensor.alpha() > 200)
        {
            return true;
        }
        return false;
    }
}