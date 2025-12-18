package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled

@TeleOp(name="TestRobot", group="Iterative Opmode")
public class TestRobot extends OpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    //Declare the wheels
    //test
    private DcMotor LF = null; //Located on Control Hub- Motor port 0
    private KitchenSink ks;
    private DcMotor RF = null; //Located on Control Hub- Motor port 2
    private DcMotor LB = null; //Located on Control Hub- Motor port 1
    private DcMotor RB = null; //Located on Control Hub- Motor port 3
    private boolean aIsPressed = false;
    double rTrig = 0.0;
    private boolean isEating = false;
    // Left trigger is upward and Right is Downward.
    private DcMotor SR = null; //Located on Control Hub- Motor port 1
   private DcMotor Sl = null; //Located on Control Hub- Motor port 3
    //private CookieShooter CS = null;
    private IMU imu = null;

    private CRServo intake = null; //Located on Control Hub- Servo port 2

    boolean disableIMU = true;




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
        SR = hardwareMap.get(DcMotor.class, "SR");
        Sl = hardwareMap.get(DcMotor.class, "SL");
        //CS = new CookieShooter(hardwareMap);
        intake  = hardwareMap.get(CRServo.class, "intake");
        intake.setPower(0.0);
       // imu = hardwareMap.get(IMU.class, "imu");
       // IMU.Parameters parameters = new IMU.Parameters(
              //  new RevHubOrientationOnRobot(
                       // RevHubOrientationOnRobot.LogoFacingDirection.UP,
                      //  RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)
      //  );

       // imu.initialize(parameters);




        //gripper sensor for pulling arm down
        //touch  = hardwareMap.get(TouchSensor .class, "Touch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.REVERSE);
        SR.setDirection(DcMotor.Direction.FORWARD);
        Sl.setDirection(DcMotor.Direction.REVERSE);
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


        //Code for gamepad1
        //Code for throttling the power factor
        PowerFactor = (1 - gamepad1.right_trigger);

        //Code for mecanum wheels

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * PowerFactor;
       // Orientation angles = imu.getRobotOrientation(AxesReference.INTRINSIC,
              //  AxesOrder.ZYX,
               // RADIANS);
        double heading =0.0;// (disableIMU) ? 0.0 : angles.firstAngle;
        //double heading =  initHeading;
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4 + heading;
        double rightX = Math.pow(gamepad1.right_stick_x, 1.0)*(1-gamepad1.right_trigger);
        double rTrig = gamepad2.right_trigger;
        LBPower = r * Math.cos(robotAngle) - rightX;
        RBPower = r * Math.sin(robotAngle) + rightX;
        LFPower = r * Math.sin(robotAngle) - rightX;
        RFPower = r * Math.cos(robotAngle) + rightX;



        // Send calculated power to wheels
        LB.setPower(LBPower);
        RB.setPower(RBPower);
        LF.setPower(LFPower);
        RF.setPower(RFPower);
        if (gamepad2.a && ! aIsPressed) {
            aIsPressed = true;
            if (isEating) {
                Actions.runBlocking(new SequentialAction(ks.rest()));
                isEating = false;
            } else {
                Actions.runBlocking(new SequentialAction(ks.eat()));
                isEating = true;
            }

        } else if (!gamepad2.a) {
            aIsPressed = false;
        }
        if (gamepad2.x) {
            intake.setPower(1.0);
        } else if (gamepad2.y) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }

        //Send telemetry data of the motor power for wheels
        telemetry.addData("Left Front Motor","Speed: "+ LFPower);
        telemetry.addData("Left Back Motor","Speed: "+ LBPower);
        telemetry.addData("Right Front Motor","Speed: "+RFPower);
        telemetry.addData("Right Back Motor","Speed: "+ RBPower);
        telemetry.addData("shooter power", rTrig);
        //telemetry.addData("Intake Motor", "Speed: = "+ intakePower);
        telemetry.addData("IMU Disable", disableIMU);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            //Nothing in stop
    }
}





