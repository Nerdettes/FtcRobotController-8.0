package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Drive Robot", group="Iterative Opmode")
public class DriveRobot extends OpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    //Declare the wheels
    //test
    private KitchenSink ks;
    private boolean bIsPressed = false;
    private boolean isEating = false;
    private boolean isIngesting = false;

    private boolean liftIsSet = false;
    private GripperWrist.WristPosition gripperPos = GripperWrist.WristPosition.Front;
    private boolean xIsPresssed = false;
    private boolean gripperIsClosed = true;
    private boolean rightBumperIsPressed = false;
    private boolean leftBumperIsPressed = false;
    private boolean isShooting = false;
    private boolean aIsPressed = false;
    private boolean isLobbing = false;



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


        if (gamepad2.b && ! bIsPressed) {
            bIsPressed = true;
            if (isShooting) {
                Actions.runBlocking(new SequentialAction(ks.cookie.cookie(0.0)));
                isShooting = false;
            } else {
                Actions.runBlocking(new SequentialAction(ks.cookie.cookie(0.5)));
                isShooting = true;
            }


        } else if (!gamepad2.b) {
            bIsPressed = false;
        }

        if (gamepad2.a && ! aIsPressed) {
            aIsPressed = true;
            if (isLobbing) {
                Actions.runBlocking(new SequentialAction(ks.cookie.cookie(0.0)));
                isLobbing = false;
            } else {
                Actions.runBlocking(new SequentialAction(ks.cookie.cookie(0.64)));
                isLobbing = true;
            }


        } else if (!gamepad2.a) {
            aIsPressed = false;
        }


        if (gamepad2.right_bumper && ! rightBumperIsPressed) {
            rightBumperIsPressed = true;
            if (isIngesting) {
                Actions.runBlocking(new SequentialAction(ks.throat.digest()));
                isIngesting = false;
            } else {
                Actions.runBlocking(new SequentialAction(ks.throat.ingest()));
                isIngesting = true;
            }

        } else if (!gamepad2.right_bumper) {
            rightBumperIsPressed = false;
        }
        if (gamepad2.left_bumper && ! leftBumperIsPressed) {
            leftBumperIsPressed = true;
            if (isEating) {
                Actions.runBlocking(new SequentialAction(ks.mouth.chew()));
                isEating = false;
            } else {
                Actions.runBlocking(new SequentialAction(ks.mouth.bite()));
                isEating = true;
            }

        } else if (!gamepad2.left_bumper) {
            leftBumperIsPressed = false;
        }

        telemetry.addData("Status","Run Time: "+runtime.toString());
       // telemetry.addData("Lift: ", ks.plate.getPosition());
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





