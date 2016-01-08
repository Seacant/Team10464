package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.ftcrobotcontroller.Map;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Travis on 10/3/2015.
 * Team 10464 Autonomous program
 */

public class AutoBlue6 extends OpMode {
    public final double TOL = 7; //tolerance for heading calculations
    public final double DEGREES_TO_FEET = 0.0004659239004629629629629629629629629629629629629629;
    // ((204.5*.03937))/(1440*12) = Constant for converting encoder = ^^^^^^^^
    //EXPLAINATION:
    //204.5=65(diameter of sprocket in mm)*PI
    //.03937 = millimeter to inch conversion
    //(1440*12) converts encoder reading, where one rotation is 1440, to feet.
    //**WARNING** Always calculate distance CHANGED, since encoders have no
    // concept of direction, and we are moving across a 2D plane.
    DcMotor motorRT;
    DcMotor motorRB;
    DcMotor motorLT;
    DcMotor motorLB;
    DcMotor motorA;
    DcMotor motorS;
    DcMotor motorC;
    GyroSensor gyro;
    Servo climber;
    Servo swingLeft;
    Servo blockRight;
    Servo blockLeft;
    Servo swingRight;


    //We stateful now, boys.
    int gameState;
    int moveState;

    double power;
    double heading;
    int cDist, lDist;
    int dDist; //the aforementioned difference (cDist-lDist) **CAN BE NEGATIVE
    double tDiff; // getRuntime() does this really annoying thing where it counts init time, so I
                  // mark the first time I exit init, and override getRuntime to return that instead
    double climbTime;


    int startPos = 6;
    Map map = new Map(startPos); //this map object will allow for easy manipulations.

    public AutoBlue6() {
        //not used in the history of ever.
    }

    @Override
    public void init() {
        motorRT = hardwareMap.dcMotor.get("motor_RT");
        motorRB = hardwareMap.dcMotor.get("motor_RB");
        motorLT = hardwareMap.dcMotor.get("motor_LT");
        motorLB = hardwareMap.dcMotor.get("motor_LB");

        motorRT.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorLT.setDirection(DcMotor.Direction.FORWARD);
        motorLB.setDirection(DcMotor.Direction.FORWARD);

        motorA = hardwareMap.dcMotor.get("motor_A");
        motorS = hardwareMap.dcMotor.get("motor_S");
        motorC = hardwareMap.dcMotor.get("motor_C");

        climber = hardwareMap.servo.get("climber");
        swingLeft = hardwareMap.servo.get("swing_l");
        swingRight = hardwareMap.servo.get("swing_r");
        blockRight = hardwareMap.servo.get("block_r");
        blockLeft = hardwareMap.servo.get("block_l");

        climber.setPosition(0);
        swingLeft.setPosition(.8);
        swingRight.setPosition(.8);
        blockRight.setPosition(0);
        blockLeft.setPosition(1);

        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    public void linedUp(int o, int n) {
        if (Math.abs(heading - map.angleToGoal()) < TOL || (heading > 360 - TOL && map.angleToGoal() < TOL || (heading < TOL && map.angleToGoal() > 360 - TOL))) {
            moveState = o;
        } else {
            moveState = n;
        }
    }

    @Override
    public double getRuntime(){
        return super.getRuntime()-tDiff;
    }

    @Override
    public void loop() {
        heading = gyro.getHeading();
        lDist = cDist;
        cDist = ( motorLB.getCurrentPosition()
                + motorRB.getCurrentPosition()
                + motorLT.getCurrentPosition()
                + motorRT.getCurrentPosition()
                ) / 4;
        dDist = cDist - lDist;

        //Goal-specific logic
        switch(gameState){
            case 0: //Start of game:
                if(tDiff == 0){tDiff = getRuntime();}
                if(getRuntime() > 5 || !gyro.isCalibrating()) {
                    gameState = 1;
                }
                break;
            case 1: //Move up before turning to beacon
                map.setGoal(startPos,9);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;
                    gameState = 2;
                }
                break;
            case 2: //Move to beacon
                map.setGoal(9.25,4);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1){
                    moveState = 0;
                    gameState = 3;
                }
                break;
            case 3: //move to climber deposit
                map.setGoal(11.5,4);
                linedUp(1,2);
                if(map.distanceToGoal() <= .1) {
                    moveState = 0;
                    gameState = 4;
                }
                break;
            case 4: // line up, and drop climbers
                map.setGoal(12, 4);
                linedUp(5,2);
                if(climbTime > 0 && getRuntime() > climbTime+1){
                    moveState = 0;
                    gameState = 5;
                }
                break;
            case 5: // Back up to avoid wall while turning
                map.setGoal(11, 4);
                if (Math.abs(heading - map.angleToGoalRev()) < TOL || (heading > 360 - TOL && map.angleToGoalRev() < TOL || (heading < TOL && map.angleToGoalRev() > 360 - TOL))) {
                    moveState = 3;
                } else {
                    moveState = 2;
                }
                if(map.distanceToGoal() <= .1){
                    moveState = 0;
                    gameState = 6;
                }
                break;
            case 6: // move to ramp alignment spot
                map.setGoal(9.5,6.5);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) {
                    blockLeft.setPosition(0);
                    blockRight.setPosition(1);
                    moveState = 0;  // stop the robot
                    gameState = 7;  // Move to the next stage.
                }
                break;
            case 7: //align with ramp, and gun it up.
                map.setGoal(47, 45);
                linedUp(1,2);
                break;
            case 777:
                moveState = 0;
                break;
        }

        if(getRuntime() > 29) gameState = 777;  //robot death switch

        switch(moveState){
            case 0:
                //Case zero is 'stop'
                motorRT.setPower(0);
                motorRB.setPower(0);
                motorLT.setPower(0);
                motorLB.setPower(0);
                break;
            //Never should we be just 'moving', always move TOWARDS something.
            case 1:
                //Case one is 'move towards' in the most literal sense. It assumes the path is
                //clear, and that there is a goal(9), and us(1) on the map somewhere.
                power = 1; //power coefficient
                if(map.distanceToGoal()>1/12) {
                    motorRT.setPower(power);
                    motorRB.setPower(power);
                    motorLT.setPower(power);
                    motorLB.setPower(power);
                    map.moveRobot(dDist * DEGREES_TO_FEET, heading);
                }
                break;
            case 2:
                //Case Two is 'turn towards'.
                power = 0.25;
                boolean turnRight;

                if(heading<=180){
                    turnRight = heading <= map.angleToGoal() && heading + 180 >= map.angleToGoal();
                }else{
                    turnRight = !(heading >= map.angleToGoal() && heading - 180 <= map.angleToGoal());
                }

                if(turnRight){
                    motorRT.setPower(-power);
                    motorRB.setPower(-power);
                    motorLT.setPower(power);
                    motorLB.setPower(power);
                }else{
                    motorRT.setPower(power);
                    motorRB.setPower(power);
                    motorLT.setPower(-power);
                    motorLB.setPower(-power);
                }
                break;
            case 3:
                climber.setPosition(0); //Move climber back to up position
                power = -1; //power coefficient
                if(map.distanceToGoal()>1/12) {
                    motorRT.setPower(power);
                    motorRB.setPower(power);
                    motorLT.setPower(power);
                    motorLB.setPower(power);
                    map.moveRobot(dDist * DEGREES_TO_FEET, heading);
                }
                break;
            case 5:
                if(climber.getPosition() < .5) climbTime = getRuntime();
                climber.setPosition(.75);
                break;
        }

        telemetry.addData("Runtime ",getRuntime());
        telemetry.addData("heading ",heading);
        telemetry.addData("goal x,y ",map.getGoalX()+","+map.getGoalY());
        telemetry.addData("robot x,y ",map.getRobotX()+","+map.getRobotY());
        telemetry.addData("angle to goal ",map.angleToGoal());
        telemetry.addData("dist from goal ",map.distanceToGoal());
    }
}
