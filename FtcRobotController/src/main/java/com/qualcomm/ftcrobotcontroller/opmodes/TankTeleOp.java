package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
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
    DcMotor motorClaw;
    Servo climber;
    Servo rightSwing;
    Servo leftSwing;
    Servo leftBlocker;
    Servo rightBlocker;
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
        motorClaw = hardwareMap.dcMotor.get("motor_C");


        motorRight.setDirection(DcMotor.Direction.FORWARD);
        motorRightB.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);
        motorLeftB.setDirection(DcMotor.Direction.REVERSE);
        
        climber = hardwareMap.servo.get("climber");
        rightSwing = hardwareMap.servo.get("swing_r");
        leftSwing = hardwareMap.servo.get("swing_l");
        rightBlocker = hardwareMap.servo.get("block_r");
        leftBlocker = hardwareMap.servo.get("block_l");

    }

    /*
     * This method will be called repeatedly in a loop, the light hitter is light_hit
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        motorArmExtender.setPower(0);
        motorArm.setPower(0);
        motorClaw.setPower(0);
        
        if(gamepad2.right_bumper){
            rightBlocker.setPosition(.2);//Moves both blockers

        }
        if(gamepad2.right_trigger > 0.5){
            rightBlocker.setPosition(1);//Moves both blockers

        }
        if(gamepad2.left_bumper){
            leftBlocker.setPosition(1);
        }
        if(gamepad2.left_trigger > 0.5){
            leftBlocker.setPosition(.3);
        }

        if(gamepad1.left_bumper){
            motorClaw.setPower(1); // Needs to be fixed -- Should operate ball collection
        }
        if(gamepad1.right_bumper){
            motorClaw.setPower(-1);
        }


        if(gamepad1.left_trigger>.1) {
            motorArmExtender.setPower(-gamepad1.left_trigger);
        }
        if(gamepad1.right_trigger>.1){
            motorArmExtender.setPower(gamepad1.right_trigger);
        }
        
        if(gamepad1.dpad_up){
            motorArm.setPower(.5);
        }
        if(gamepad1.dpad_down){
            motorArm.setPower(-0.5);
        }
        if(gamepad2.dpad_left){
            rightSwing.setPosition(.75);
        }

        if(gamepad2.dpad_right){
            rightSwing.setPosition(.55); //1 Up, 0 Down
        }

        if(gamepad2.y){
            climber.setPosition(climber.getPosition()>.5?0:1); //moves climber
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
