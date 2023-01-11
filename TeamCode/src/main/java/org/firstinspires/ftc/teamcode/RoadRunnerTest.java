package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//test1
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
//test

@Autonomous(name = "RoadRunnerTest", group = "")
public class RoadRunnerTest extends ActionOpMode {
    public static double MOTOR_POWER = 0.7;
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    private DcMotor arm = null;

    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!

    public MecanumDrive drive;
    public Telemetry telemetry;
    private  OpenCvCamera webCam;
    private boolean isCameraStreaming = false;
    Pipeline modifyPipeline = new Pipeline();
    private int resultROI=2;
    private  boolean done = false;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            throw new RuntimeException(getClass().getSimpleName() + " is for mecanum drives only.");
        }

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Press play to begin the debugging op mode");
        telemetry.update();

        waitForStart();

        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);

        /*
        // IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        */

        actuatorUtils.initializeActuator(arm, gripper);

        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        initOpenCV();

        actuatorUtils.gripperClose(false);


        waitForStart();
        currTime=System.currentTimeMillis();
        startTime=currTime;
        if (resultROI == 2) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            done = false;
            while (!done && opModeIsActive()) {
                if (currTime - startTime < 500) {
                    telemetry.addData("Camera: ", "Waiting to make sure valid data is incoming");
                } else {
                    telemetry.addData("Time Delta: ", (currTime - startTime));
                    resultROI = modifyPipeline.getResultROI();
                    if (resultROI == 1) {
                        telemetry.addData("Resulting ROI: ", "Red");
                        done = true;
                    } else if (resultROI == 2) {
                        telemetry.addData("Resulting ROI: ", "Green");
                        done = true;
                    } else if (resultROI == 3) {
                        telemetry.addData("Resulting ROI: ", "Blue");
                        done = true;
                    } else {
                        telemetry.addData("Resulting ROI: ", "Something went wrong.");
                    }
                }
                telemetry.update();
                currTime = System.currentTimeMillis();

            }

        }
        telemetry.update();
        done = false;
        //lift arm up
        actuatorUtils.armPole(4);
        while (((currTime - startTime) < 30000)&& !done && opModeIsActive()) {

            switch (resultROI) {
                case 1:
                    // Far left
                    runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(30, 30), Math.PI / 2)
                                    .splineTo(new Vector2d(60, 0), Math.PI)
                                    .build());
                    actuatorUtils.gripperOpen(true);
                    done=true;
                    break;
                case 2:
                    // Middle
                    runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(25, 30), Math.PI / 2)
                                    .splineTo(new Vector2d(60, 0), Math.PI)
                                    .build());
                    actuatorUtils.gripperOpen(true);
                    done=true;
                    break;
                case 3:
                    // Far right
                    runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .splineTo(new Vector2d(30, 25), Math.PI / 2)
                                    .splineTo(new Vector2d(60, 0), Math.PI)
                                    .build());
                    actuatorUtils.gripperOpen(true);
                    done=true;
                    break;
            }

            currTime = System.currentTimeMillis();

        }
    }


    private void initOpenCV() {
        int cameraMonitorViewId2 = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        // For a webcam (uncomment below)
        webCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId2);
        // For a phone camera (uncomment below)
        // webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId2);
        webCam.setPipeline(modifyPipeline);
        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Pipeline: ", "Initialized");
                telemetry.update();
                isCameraStreaming = true;
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Error: ", "Something went wrong :(");
                telemetry.update();
            }
        });
    }
}
