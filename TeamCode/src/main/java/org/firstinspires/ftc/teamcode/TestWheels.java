package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled

@TeleOp(name="TestWheels", group="Iterative Opmode")
public class TestWheels extends OpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    //Declare the wheels
    //test
    private DcMotor LF = null; //Located on Control Hub- Motor port 0
    private DcMotor RF = null; //Located on Control Hub- Motor port 2
    private DcMotor LB = null; //Located on Control Hub- Motor port 1
    private DcMotor RB = null; //Located on Control Hub- Motor port 3
    // Left trigger is upward and Right is Downward.





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




        //gripper sensor for pulling arm down
        //touch  = hardwareMap.get(TouchSensor .class, "Touch");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
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





        if (gamepad1.a) {
            RF.setPower(1.0);
            LF.setPower(0.0);
            LB.setPower(0.0);
            RB.setPower(0.0);
        } else if (gamepad1.b) {
            LF.setPower(1.0);
            RF.setPower(0.0);
            LB.setPower(0.0);
            RB.setPower(0.0);
        } else if (gamepad1.x) {
            LB.setPower(1.0);
            RF.setPower(0.0);
            LF.setPower(0.0);
            RB.setPower(0.0);
        } else if (gamepad1.y) {
            RB.setPower(1.0);
            RF.setPower(0.0);
            LF.setPower(0.0);
            LB.setPower(0.0);
        } else {
            RF.setPower(0.0);
            LF.setPower(0.0);
            LB.setPower(0.0);
            RB.setPower(0.0);
        }


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
            //Nothing in stop
    }
}





