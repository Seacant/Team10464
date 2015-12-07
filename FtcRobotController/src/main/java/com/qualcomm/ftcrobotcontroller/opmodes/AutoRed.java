package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.ftcrobotcontroller.Map;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;


/**
 * Created by Travis on 10/3/2015.
 * Team 10464 Autonomous program
 */

public class AutoRed extends OpMode {
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
    GyroSensor gyro;
    ColorSensor color;
    OpticalDistanceSensor ODSC;
    OpticalDistanceSensor ODSR;
    OpticalDistanceSensor ODSL;
    UltrasonicSensor USM; //UltraSonic Middle
    Servo climber;
    Servo swingLeft;
    Servo swingRight;


    //We stateful now, boys.
    int gameState;
    int moveState;
    double power;
    double sTime;
    double eTime;
    double dTime;
    int cDist; //current distance (from encoder) reading
    int lDist; //last distance (from encoder) reading
    int dDist; //the aforementioned difference (cDist-lDist) **CAN BE NEGATIVE

//Map visualization
//      {0,0,0,0,0,0,0,0,3,3,3,3}
//      {0,0,0,0,0,0,0,0,0,3,3,3}
//      {0,0,0,0,0,0,0,0,0,0,3,3}
//      {0,0,0,0,0,0,0,0,0,0,0,3}
//      {0,0,0,0,0,0,0,0,0,0,0,0}
//      {0,0,0,0,0,0,0,0,0,0,0,0}
//      {0,0,0,0,0,0,0,0,0,0,0,0}
//      {3,0,0,0,0,0,0,0,0,0,0,0}
//      {3,3,0,0,0,0,0,0,0,0,0,0}
//      {3,3,3,0,0,0,0,0,0,0,0,0}
//      {3,3,3,3,0,0,0,0,0,0,0,0}

    Map map = new Map("Red"); //this map object will allow for easy manipulations.

    //GYRO
    double xVal,yVal,zVal,heading;

    public AutoRed() {
        //not used in the history of ever.
    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        motorRT = hardwareMap.dcMotor.get("motor_RT");
        motorRB = hardwareMap.dcMotor.get("motor_RB");

        motorLT = hardwareMap.dcMotor.get("motor_LT");
        motorLB = hardwareMap.dcMotor.get("motor_LB");

        motorRT.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);

        motorA = hardwareMap.dcMotor.get("motor_A");
        motorS = hardwareMap.dcMotor.get("motor_S");
        climber = hardwareMap.servo.get("climber");
        swingLeft = hardwareMap.servo.get("swing_l");
        swingRight = hardwareMap.servo.get("swing_r");

        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        //Information gathering phase
        sTime = eTime;
        eTime = getRuntime();
        dTime = eTime - sTime;
        lDist = cDist;
        cDist = (motorLB.getCurrentPosition()+motorRB.getCurrentPosition()+motorLT.getCurrentPosition()+motorRT.getCurrentPosition())/4; //average of motor positions
        dDist = cDist-lDist;
        heading = gyro.getHeading();

        //Constantly checks the state of ultrasonic sensors. If an object is too close and we are not next to the mountains or one of the boundaries of the course, then we shift to moveState 3 (going around something).


        //Goal-specific logic
        switch(gameState){
            case 0: //Start of game:
                //It was recommended to us that we should wait at the gate for a few second to allow
                //our teammate to GTFO, avoiding unnecessary beginning-game collisions. It will also
                //give our gyro a second to calibrate.
                if(getRuntime() > 5 || !gyro.isCalibrating()) {
                    climber.setPosition(1);
                    gameState = 1;
                }
                break;
            case 1:
                map.setGoal(2,6);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1) { //TODO: '|| colorsensor = white'
                    moveState = 0;  // stop the robot
                    gameState = 2;  // Move to the next stage.
                }
                break;
            case 2: //Move to beacon
                //// TODO: 10/27/2015 Expand on gameState 1 uses
                map.setGoal(6, 1.75);
                //Checks our heading.
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1) { //TODO: '|| colorsensor = white'
                    moveState = 0;  // stop the robot
                    gameState = 3;  // Move to the next stage.
                }
                break;
            case 3:
                map.setGoal(6, 0.75);
                //Checks our heading.
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1) { //TODO: '|| colorsensor = white'
                    moveState = 0;  // stop the robot
                    gameState = 4;  // Move to the next stage.
                }
                break;
            case 4:
                map.setGoal(6,0);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 4 : 2;
                if(Math.abs(climber.getPosition()-.25) < .02){
                    moveState = 0;
                    gameState = 5;
                }
                break;
            case 5:
                map.setGoal(3.5,2.5);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1) { //TODO: '|| colorsensor = white'
                    moveState = 0;  // stop the robot
                    gameState = 6;  // Move to the next stage.
                }
                break;
            case 6:
                map.setGoal(-26,-22.5);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                break;
        }
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
                if(heading-map.angleToGoal()>0) {
                    motorRT.setPower(power);
                    motorRB.setPower(power);
                    motorLT.setPower(-power);
                    motorLB.setPower(-power);
                }else{
                    motorRT.setPower(-power);
                    motorRB.setPower(-power);
                    motorLT.setPower(power);
                    motorLB.setPower(power);
                }
                break;
            case 3:
                //Case Three is 'move around'. implies there is something in front of us that we'd
                //like to not hit.

                // Checks our current position and orientation to make sure we are not in the mountain zone or facing one of the boundaries of the course. (Redundant but good to have)

                // Stop, wait for .5 seconds and recheck if object is still there. This might help us avoid starting to turn when a robot is about to get out of the way as well to avoid the start of a turn due to a false detection of an object.

                // Detects proximity of the robot, randomly chooses a direction, then makes a small rotation towards the chosen direction so that if a robot of given dimensions (max dimensions) was in front the sensor would not be able to detect it any more. Checks if object is still in front. If it is, then move in the direction opposite to that chosen. If it isn’t move forward until it is safe to change direction back to the original objective.

                break;
            case 4:
                climber.setPosition(.25);
                break;
        }

        telemetry.addData("Runtime ",getRuntime());
        telemetry.addData("heading ",heading);
        telemetry.addData("goal x,y ",map.getGoalX()+","+map.getGoalY());
        telemetry.addData("robot x,y ",map.getRobotX()+","+map.getRobotY());
        telemetry.addData("angle to goal ",map.angleToGoal());
        telemetry.addData("dist from goal ",map.distanceToGoal());
        telemetry.addData("moveState & gameState ",moveState + " " + gameState);
        telemetry.addData("climber pos: ", climber.getPosition());
        telemetry.addData("ODS left", climber.getPosition());
        telemetry.addData("ODS center", "Test");
        telemetry.addData("ODS right", climber.getPosition());
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