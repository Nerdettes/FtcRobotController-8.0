package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
    private KitchenSink ks;
    private boolean bIsPressed = false;
    private boolean liftIsSet = false;
    private GripperWrist.WristPosition gripperPos = GripperWrist.WristPosition.Front;
    private boolean xIsPresssed = false;
    private boolean gripperIsClosed = true;
    private boolean rightBumperIsPressed = false;



     private SequentialAction handoff;

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
        ks = new KitchenSink(hardwareMap, startPose);

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

        ks.drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -0.8*gamepad1.left_stick_y,
                        -0.8*gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x
        ));

        ks.drive.updatePoseEstimate();

        Actions.runBlocking(new SequentialAction(ks.intake.intakeRun(gamepad2.left_stick_y)));

        Actions.runBlocking(new SequentialAction(ks.intakeSlide.intakeRun(-gamepad2.right_stick_y)));

        if (gamepad2.right_bumper && ! rightBumperIsPressed){
            rightBumperIsPressed = true;
            Actions.runBlocking(ks.handoff());
        } else if (!gamepad2.right_bumper){
            rightBumperIsPressed= false;

        }

        if (gamepad2.b && ! bIsPressed) {
            bIsPressed = true;
            if (gripperPos == GripperWrist.WristPosition.Front) {
                Actions.runBlocking(new SequentialAction(ks.gripperWrist.wristBack()));
                gripperPos = GripperWrist.WristPosition.Back;
            } else if (gripperPos == GripperWrist.WristPosition.Wall) {
                Actions.runBlocking(new SequentialAction(ks.gripperWrist.wristFront()));
                gripperPos = GripperWrist.WristPosition.Front;
            } else {
                Actions.runBlocking(new SequentialAction(ks.gripperWrist.wristWall()));
                gripperPos = GripperWrist.WristPosition.Wall;
            }
        } else if (!gamepad2.b) {
            bIsPressed = false;
        }

        if (gamepad2.x && ! xIsPresssed) {
            xIsPresssed = true;
            if (gripperIsClosed)
                Actions.runBlocking(new SequentialAction(ks.gripper.gripperOpen()));
            else
                Actions.runBlocking(new SequentialAction(ks.gripper.gripperClosed()));
            gripperIsClosed = ! gripperIsClosed;
        } else if (!gamepad2.x) {
            xIsPresssed = false;
        }
        if (gamepad2.y) {
            Actions.runBlocking(new SequentialAction(ks.wrist.rotateUp()));
        }  else if (gamepad2.a) {
            Actions.runBlocking(new SequentialAction(ks.wrist.rotateDown()));
        }
        if (gamepad2.right_trigger > 0.1) {
            Actions.runBlocking(new SequentialAction(ks.lift.liftRun(gamepad2.right_trigger)));
            liftIsSet = false;
        } else if (gamepad2.left_trigger > 0.1) {
            Actions.runBlocking(new SequentialAction(ks.lift.liftRun(-gamepad2.left_trigger)));
            liftIsSet = false;
        } else if (!liftIsSet){
            int pos = ks.lift.getPosition();
            Actions.runBlocking(new SequentialAction(ks.lift.setPosition(pos, 0.05)));
            liftIsSet = true;
        }
        if (gamepad2.left_bumper) {
            ks.reset();
        }
        if (gamepad2.dpad_up) {
            Actions.runBlocking(new SequentialAction(ks.gripperWrist.rotateUp()));
        }
        else if (gamepad2.dpad_down) {
            Actions.runBlocking(new SequentialAction(ks.gripperWrist.rotateDown()));
        }
        telemetry.addData("Status","Run Time: "+runtime.toString());
        telemetry.addData("Intake: ", ks.intakeSlide.getPosition());
        telemetry.addData("Lift: ", ks.lift.getPosition());
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





