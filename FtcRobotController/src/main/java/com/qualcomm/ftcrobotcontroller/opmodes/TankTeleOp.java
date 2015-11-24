package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        //motorRightB.setDirection(DcMotor.Direction.REVERSE);


    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        motorSpindle.setPower(0);
        motorArm.setPower(0);

        if(gamepad1.a){
            armPos = 1;
        }
        if(gamepad1.b){
            armPos = 0;
        }

        if(gamepad1.left_trigger>0) {
            motorSpindle.setPower(gamepad1.left_trigger);
        }
        if(gamepad1.right_trigger>0){
            motorSpindle.setPower(-1*gamepad1.right_trigger);
        }

        if(gamepad1.dpad_up) {
            motorArm.setPower(.3);
        }

        if(gamepad1.dpad_down) {
            motorArm.setPower(-.1);
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