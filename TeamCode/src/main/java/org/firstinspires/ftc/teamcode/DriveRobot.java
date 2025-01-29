package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drive Robot", group="Iterative Opmode")
public class DriveRobot extends OpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    //Declare the wheels
    //test
    private PinpointDrive drive;
    private Intake intake;
    private IntakeSlide intakeSlide;
    private Wrist wrist;
    private GripperWrist gripperWrist;
    private Gripper gripper;
    private boolean aIsPressed = false;
    private boolean wristIsUp = true;
    private boolean bIsPressed = false;
    private boolean gripperIsFront = true;
    private boolean xIsPresssed = false;
    private boolean gripperIsClosed = true;
    private boolean yIsPresssed = false;
    private boolean slideStart = true;


    //private Servo elbow = null; //Located on Expansion Hub- Servo port 0
    //private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    //private Servo plane = null; //Located on Expansion Hub- Servo port 0

    //private IMU imu = null;
    //private fileUtils fUtils;

    //variable for Rev Touch Sensor
    //private TouchSensor touch;

    Double initHeading = 0.0;

    //boolean touchIsPressed = false;

    /*
      Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        Pose2d startPose = new Pose2d(0, 0,0);
        drive = new PinpointDrive(hardwareMap, startPose);
        intake = new Intake(hardwareMap);
        intakeSlide = new IntakeSlide(hardwareMap);
        wrist = new Wrist(hardwareMap);
        gripperWrist = new GripperWrist(hardwareMap);
        gripper = new Gripper(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void loop() {

        //Code for mecanum wheels

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        drive.updatePoseEstimate();

        Actions.runBlocking(new SequentialAction(intake.intakeRun(gamepad2.left_stick_y)));

        Actions.runBlocking(new SequentialAction(intakeSlide.intakeRun(-gamepad2.right_stick_y)));

        if (gamepad2.a && ! aIsPressed) {
            aIsPressed = true;
            if (wristIsUp)
                Actions.runBlocking(new SequentialAction(wrist.wristDown()));
            else
                Actions.runBlocking(new SequentialAction(wrist.wristUp()));
            wristIsUp = ! wristIsUp;
        } else if (!gamepad2.a) {
            aIsPressed = false;
        }

        if (gamepad2.b && ! bIsPressed) {
            bIsPressed = true;
            if (gripperIsFront)
                Actions.runBlocking(new SequentialAction(gripperWrist.wristBack()));
            else
                Actions.runBlocking(new SequentialAction(gripperWrist.wristFront()));
            gripperIsFront = ! gripperIsFront;
        } else if (!gamepad2.b) {
            bIsPressed = false;
        }

        if (gamepad2.x && ! xIsPresssed) {
            xIsPresssed = true;
            if (gripperIsClosed)
                Actions.runBlocking(new SequentialAction(gripper.gripperOpen()));
            else
                Actions.runBlocking(new SequentialAction(gripper.gripperClosed()));
            gripperIsClosed = ! gripperIsClosed;
        } else if (!gamepad2.x) {
            xIsPresssed = false;
        }
        if (gamepad2.y && ! yIsPresssed) {
            yIsPresssed = true;
            if (slideStart)
                Actions.runBlocking(new SequentialAction(intakeSlide.setPosition(500)));
            else
                Actions.runBlocking(new SequentialAction(intakeSlide.setPosition(0)));
            slideStart = ! slideStart;
        } else if (!gamepad2.y) {
            yIsPresssed = false;
        }
        telemetry.addData("Status","Run Time: "+runtime.toString());
        telemetry.addData("Intake: ", intakeSlide.getPosition());
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            //Nothing in stop
    }
}





