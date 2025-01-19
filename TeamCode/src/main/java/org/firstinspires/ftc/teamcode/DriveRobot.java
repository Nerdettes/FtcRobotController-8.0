package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    // Left trigger is upward and Right is Downward.
    // positive Y on right joystick will be intake, negative = outtake
    private CRServo intakeLeft = null; //Located on Expansion Hub- Servo port 2
    private CRServo intakeRight = null; //Located on Expansion Hub- Servo port 2


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
        // Initiali ze the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        intakeLeft  = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        intakeLeft.setPower(0.0);
        intakeRight.setPower(0.0);

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

        intakeLeft.setPower(gamepad2.left_stick_y);
        intakeRight.setPower(-gamepad2.left_stick_y);
        telemetry.addData("intakeLeft:", intakeLeft.getPower());
        telemetry.addData("intakeRight:", intakeRight.getPower());
        telemetry.addData("Status","Run Time: "+runtime.toString());

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





