package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Disabled

@TeleOp(name="Competition Teleop2024NewRobot", group="Iterative Opmode")
public class CompetitionTeleop2024NewRobot extends OpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    //Declare the wheels
    //test
    private DcMotor LF = null; //Located on Control Hub- Motor port 0
    private DcMotor RF = null; //Located on Control Hub- Motor port 2
    private DcMotor LB = null; //Located on Control Hub- Motor port 1
    private DcMotor RB = null; //Located on Control Hub- Motor port 3
    // Left trigger is upward and Right is Downward.
    private DcMotor lift = null; // location on expansion hub - Motor port 0
    //Declare variables used for our arm lift

    // positive Y is upward on joystick, left and right
    private DcMotor arm = null; //Located on Expansion Hub- Motor port 1
    private Servo wrist = null; //Located on Expansion Hub- Servo port 0
    // positive Y on right joystick will be intake, negative = outtake
    private CRServo intake = null; //Located on Expansion Hub- Servo port 2

    private IMU imu = null;


    boolean autoLift = false;
    boolean disableIMU = true;
    boolean game2back = false; //Used to override switch for arm in case of failure
    boolean rightBumperPressed = false;
    boolean leftBumperPressed = false;
    boolean game1Bpressed = false;
    boolean game2Bpressed = false;

    Double initHeading = 0.0;

    //boolean touchIsPressed = false;

    /*
      Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        fileUtils fUtils;

        telemetry.addData("Status", "Initialized");

        // Initiali ze the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        intake = hardwareMap.get(CRServo.class, "intake");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lift = hardwareMap.get(DcMotor.class, "lift");
        wrist = hardwareMap.get(Servo.class, "wrist");

        intake.setPower(0.0);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
        );

        imu.initialize(parameters);

        fUtils = new fileUtils();
        fUtils.readConfig(hardwareMap.appContext, this);
        Pose2d pose = fUtils.getPose();
        initHeading = pose.heading.toDouble();


        //gripper sensor for pulling arm down
        //touch  = hardwareMap.get(TouchSensor .class, "Touch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setPower(0);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setPower(0);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    /*
     * Code to run ONCE when the driver hits PLAY
     */

    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;
        double PowerFactor; //Max power available for wheels
        double armPower = 1.0f;

        //Code for gamepad1
        //Code for throttling the power factor
        PowerFactor = (1 - gamepad1.right_trigger);

        //Code for mecanum wheels

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * PowerFactor;
        Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                RADIANS);
        double heading = (disableIMU) ? 0.0 : angles.firstAngle;
        //double heading =  initHeading;
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 + heading;
        double rightX = Math.pow(gamepad1.right_stick_x, 1.0)*(1-gamepad1.right_trigger);
        LBPower = r * Math.cos(robotAngle) - rightX;
        RBPower = r * Math.sin(robotAngle) + rightX;
        LFPower = r * Math.sin(robotAngle) - rightX;
        RFPower = r * Math.cos(robotAngle) + rightX;



        // Send calculated power to wheels
        LB.setPower(LBPower);
        RB.setPower(RBPower);
        LF.setPower(LFPower);
        RF.setPower(RFPower);



        //Send telemetry data of the motor power for wheels
        telemetry.addData("Left Front Motor","Speed: "+ LFPower);
        telemetry.addData("Left Back Motor","Speed: "+ LBPower);
        telemetry.addData("Right Front Motor","Speed: "+RFPower);
        telemetry.addData("Right Back Motor","Speed: "+ RBPower);
        //telemetry.addData("Intake Motor", "Speed: = "+ intakePower);
        telemetry.addData("Arm Encoder Height","Height: "+arm.getCurrentPosition());
        telemetry.addData("IMU Disable", disableIMU);
        telemetry.addData("Initial Heading", "Heading: "+initHeading);

        //Code for gamepad2
        //Toggle auto and manual mode

        if (gamepad2.back) {
            game2back = !game2back;
        }




        //Moves the arm up
        if (gamepad2.left_trigger >= .1)
        {
            autoLift = false;
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(-gamepad2.left_trigger * (0.5));
            telemetry.addData("Lift ticks", lift.getCurrentPosition());
            telemetry.update();
            //Moves the arm down
        }
        else if (gamepad2.right_trigger >= .1)
        {
            autoLift = false;
            // Hard Stop
            if (lift.getCurrentPosition() < 3380) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lift.setPower(gamepad2.right_trigger * (0.5));
            } else {
                lift.setPower(0);
            }
        }
        else if (gamepad2.y) {
            autoLift = true;
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(-0.5);
        }
        else if (gamepad2.x) {
            autoLift = false;

        }
        else if (gamepad2.b) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setPower(0.0);

        }

        else if (!autoLift)
        {
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setPower(0.0);

        }

        //Code to increase height position
        if (gamepad1.b & !game1Bpressed) {
            game1Bpressed = true;
            disableIMU = !disableIMU;
        } else if (!gamepad1.b) {
            game1Bpressed = false;
        }

        if (gamepad1.a) {
            initHeading = 0.0;
            imu.resetYaw();
        }
        if (gamepad2.b & !game2Bpressed) {
            game2Bpressed = true;
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setPower(0.0);
        } else if (!gamepad2.b & game2Bpressed) {
            game2Bpressed = false;
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            //Allows the drivers to use a single button to open and close gripper
            arm.setPower(-gamepad2.right_stick_y * armPower);
        }



        intake.setPower(gamepad2.left_stick_y);
        if (gamepad2.right_bumper && ! rightBumperPressed) {
            rightBumperPressed = true;
            wrist.setPosition(0.333);
        }
        else if (!gamepad2.right_bumper) {
            rightBumperPressed = false;
        }

        if (gamepad2.left_bumper && !leftBumperPressed) {
            leftBumperPressed = true;
            wrist.setPosition(0.0);
        } else if (!gamepad2.left_bumper) {
            leftBumperPressed = false;
        }

        if (gamepad2.a) {
            wrist.setPosition(1.0);
        }
        // Show the elapsed game time and wheel power.
        telemetry.addData("arm: ",arm.getCurrentPosition());
        telemetry.addData("lift: ",lift.getCurrentPosition());
        telemetry.addData("intake:", intake.getPower());
        telemetry.addData("wrist:", wrist.getPosition());
        telemetry.addData("Status","Run Time: "+runtime.toString());
        //telemetry.addData("touchIsPressed ", touchIsPressed);
        telemetry.update();
        //telemetry.addData("positionTarget: ", "%.2f", positionTarget);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            //Nothing in stop
    }
}





