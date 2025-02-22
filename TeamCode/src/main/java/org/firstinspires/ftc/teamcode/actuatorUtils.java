package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class actuatorUtils {

    private static DcMotor lift = null; //declare arm

    private static DcMotor arm = null; //declare gripper
    private static CRServo intake = null; //declare dump
    private static Servo wrist = null;
    //test
    public static int upEncode = 1650; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    public static int restEncode = 1180; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    public static int downEncode = 0; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    public static double armPower = 1.0; //Set power to .7 so arm does not go up too fast
    public static int maxEncode = 3400; //4200 for higher, 2175 for lower-- Max so arm won't overextend and position 3
    public static int minEncode = 0; //Minimum so string on arm lift doesn't break and position 0
    public static int lowEncode = 1600; //Minimum so string on arm lift doesn't break and position 0
    public static int highEncode = maxEncode; //Minimum so string on arm lift doesn't break and position 0
    public static double liftPower = .7f; //Set power to .7 so arm does not go up too fast
    public static int parkEncode = 1000;
   public enum LiftLevel
    {
        ZERO,
        PARK,
        LOW_BASKET,
        HIGH_BASKET

    }
    public enum IntakeModes
    {
        OFF,
        IN,
        OUT

    }
    public enum WristModes
    {
        UP,
        DOWN,
        BACK

    }
    public enum ArmModes
    {
        UP,
        DOWN,
        REST

    }
    //Initialize actuators
    public static void initializeActuator(DcMotor lift, DcMotor arm,  CRServo intake, Servo wrist) {
        actuatorUtils.lift = lift;
        actuatorUtils.arm = arm;
        actuatorUtils.intake = intake;
        actuatorUtils.wrist = wrist;
    }


    //Method used to close gripper
    public static void setIntake(IntakeModes mode)  {
        if (mode == IntakeModes.IN) {
            intake.setPower(-1.0);
        } else if (mode == IntakeModes.OUT) {
           intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }
    }
    public static void setArm(int delta) {
        int getPosition = arm.getCurrentPosition();
        arm.setTargetPosition(getPosition + delta);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
    }
    public static void setArm(ArmModes mode)   {
        if (mode == ArmModes.UP) {
            arm.setTargetPosition(upEncode); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);



        } else if (mode == ArmModes.REST) {
            arm.setTargetPosition(restEncode); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);


        } else {
            arm.setTargetPosition(downEncode); //Lifts arm up so we can move w/o drag
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(armPower);


        }
    }
    public static void setWrist(WristModes mode)  {
        if (mode == WristModes.DOWN) {
            wrist.setPosition(0.333);
        } else if (mode == WristModes.BACK) {
            wrist.setPosition(1.000);
        } else {
            wrist.setPosition(0.0);
        }
    }
    public static void setLift(int delta) {
        int getPosition = lift.getCurrentPosition();
        lift.setTargetPosition(getPosition + delta);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(liftPower);
    }
    public static void setLift(LiftLevel mode) {
        if (mode == LiftLevel.ZERO) {
            lift.setTargetPosition(minEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }  else if (mode == LiftLevel.PARK) {
            lift.setTargetPosition(parkEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }  else if (mode == LiftLevel.LOW_BASKET) {
            lift.setTargetPosition(lowEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        } else {
            lift.setTargetPosition(highEncode); //Lifts arm up so we can move w/o drag
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setPower(liftPower);
        }
    }


}
