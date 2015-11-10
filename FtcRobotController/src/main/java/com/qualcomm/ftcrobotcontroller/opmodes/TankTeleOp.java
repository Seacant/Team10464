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
    DcMotor motorSpindle;
    Servo arm;
    float armPos;
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
        motorSpindle = hardwareMap.dcMotor.get("motor_S");
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorRightB.setDirection(DcMotor.Direction.REVERSE);

        arm = hardwareMap.servo.get("servo_A");

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        if(gamepad1.a){
            armPos = 1;
        }
        if(gamepad1.b){
            armPos = 0;
        }
        if(gamepad1.dpad_left){
            motorArm.setPower(.75);
        }
        if(gamepad1.dpad_right){
            motorArm.setPower(-.75);
        }
        motorRight.setPower(gamepad1.right_stick_y);
        motorRightB.setPower(gamepad1.right_stick_y);
        motorLeft.setPower(gamepad1.left_stick_y);
        motorLeftB.setPower(gamepad1.left_stick_y);
        arm.setPosition(armPos);
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