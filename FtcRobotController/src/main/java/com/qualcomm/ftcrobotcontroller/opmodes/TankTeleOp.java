package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by Travis on 10/3/2015.
 */
public class TankTeleOp extends OpMode {

    DcMotor motorRight;
    DcMotor motorRightB;
    DcMotor motorLeft;
    DcMotor motorLeftB;
    DcMotor motorArm;
    DcMotor motorArmExtender;
    //DcMotor motorClaw;
    Servo climber;
    Servo rightSwing;
    Servo leftSwing;
    //Servo leftBlocker;
    //Servo rightBlocker;
    /**
     * Constructor
     */
    public TankTeleOp() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motor_RT");
        motorRightB = hardwareMap.dcMotor.get("motor_RB");

        motorLeft = hardwareMap.dcMotor.get("motor_LT");
        motorLeftB = hardwareMap.dcMotor.get("motor_LB");

        motorArm = hardwareMap.dcMotor.get("motor_A");
        motorArmExtender = hardwareMap.dcMotor.get("motor_S");
        //motorClaw = hardwareMap.dcMotor.get("motor_c");

        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorRightB.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        
        climber = hardwareMap.servo.get("climber");
        rightSwing = hardwareMap.servo.get("swing_r");
        leftSwing = hardwareMap.servo.get("swing_l");
        //rightBlocker = hardwareMap.servo.get("right_block");
        //leftBlocker = hardwareMap.servo.get("left_block");

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        motorArmExtender.setPower(0);
        motorArm.setPower(0);
        
        if(gamepad1.a){
            //rightBlocker.setPosition(rightBlocker.getPosition()<.5?.8:.2); //needs to be tested
        }
        if(gamepad1.b){
            //leftBlocker.setPosition(leftBlocker.getPosition()<.5?.8:.2); //needs to be tested
        }

        if(gamepad1.left_bumper){
            //motorClaw.setPower(1);
        }
        if(gamepad1.right_bumper){
            //motorClaw.setPower(-1);
        }

        if(gamepad1.left_trigger>0) {
            motorArmExtender.setPower(gamepad1.left_trigger);
        }
        if(gamepad1.right_trigger>0){
            motorArmExtender.setPower(-gamepad1.right_trigger);
        }
        
        if(gamepad1.dpad_up) {
            motorArm.setPower(.3);
        }
        if(gamepad1.dpad_down) {
            motorArm.setPower(-.3);
        }
        if(gamepad1.dpad_left){
            leftSwing.setPosition(leftSwing.getPosition()<.5?1:0);
        }

        if(gamepad1.dpad_right){
            rightSwing.setPosition(rightSwing.getPosition()<.5?1:0);
        }

        if(gamepad1.x){
            climber.setPosition(1);
        }   
        if(gamepad1.y){
            climber.setPosition(0);
        }

        motorRight.setPower(gamepad1.right_stick_y);
        motorRightB.setPower(gamepad1.right_stick_y);
        motorLeft.setPower(gamepad1.left_stick_y);
        motorLeftB.setPower(gamepad1.left_stick_y);
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

    }

}
