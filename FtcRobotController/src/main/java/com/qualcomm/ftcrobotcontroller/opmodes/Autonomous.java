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

public class Autonomous extends OpMode {
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
    double sTime; //StartTime
    double eTime; //EndTime
    double dTime; //DeltaTime
    int cDist; //current distance (from encoder) reading
    int lDist; //last distance (from encoder) reading
    int dDist; //the aforementioned difference (cDist-lDist) **CAN BE NEGATIVE

    //Avoidance vars
    double usmLevel;
    int metaGameState = -1; //GameState is not always truthful, since we enter a new state when moving around.
    //-1 = not set; else, use this as the gamestate to 'get back to', after we finish avoidance
    double aTimeStart; //AvoidTimeStart.
    double aDistTrav;  //Avoidance Distance Traveled.
    double aDistToTrav; //Avoidance Distance To Travel.
    boolean aPrefDir; //Avoidance Preferred direction. True=left & False=right
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

    Map map = new Map("Blue",6); //this map object will allow for easy manipulations.

    //GYRO
    double xVal,yVal,zVal,heading;

    public Autonomous() {
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

        color = hardwareMap.colorSensor.get("color");
        USM = hardwareMap.ultrasonicSensor.get("sonic");
        gyro = hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    public void avoid(){ //always run right before break statement, to prevent over-revision.
        if(usmLevel < 30.48){ //If something is ~a foot away, try to move around it.
            moveState = 0; // We always set moveState to 0 when changing gameStates.
            gameState = 10; //avoid
        }
    }
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

        usmLevel = USM.getUltrasonicLevel(); //Uses cm

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
            case 1: //Move up before turning to beacon
                map.setGoal(6,9);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;  // stop the robot
                    gameState = 2;  // Move to the next stage.
                }
                break;
            case 2: //Move to beacon
                map.setGoal(9.25, 6);
                //Checks our heading.
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;  // stop the robot
                    gameState = 3;  // Move to the next stage.
                }
                aPrefDir = true; //left. We need to be extremely careful with crossing over midline.
                avoid();
                break;
            case 3: //move to climber deposit
                map.setGoal(10.25, 6);
                //Checks our heading.
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1 || usmLevel < 5) {
                    moveState = 0;  // stop the robot
                    gameState = 4;  // Move to the next stage.
                }
                break;
            case 4: // line up, and drop climbers
                map.setGoal(11,6);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 5 : 2;
                if(Math.abs(climber.getPosition()-.25) < .02){
                    moveState = 0;
                    gameState = 5;
                }
                break;
            case 5: // move to ramp alignment spot
                map.setGoal(8.5,7);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                if(map.distanceToGoal()<=.1) { //TODO: '|| colorsensor = white'
                    moveState = 0;  // stop the robot
                    gameState = 6;  // Move to the next stage.
                }
                aPrefDir = false; //Right is better for us.
                avoid(); //may act erratically since we start on the wall.
                break;
            case 6: //align with ramp, and gun it up.
                map.setGoal(53,45);
                moveState = Math.abs(heading-map.angleToGoal()) < TOL ? 1 : 2;
                break;
            case 10: // Move Around.
                if(metaGameState == -1){
                    metaGameState = gameState; //save my starting game state
                    aTimeStart = sTime;
                    aDistTrav = 0;
                    aDistToTrav = map.distanceToGoal() / 8; //Arbitrary denominator. Change to fit testing.
                }
                if(aTimeStart > 2) { //give it a second before we do stuff.
                    // Detects proximity of the robot, chooses a direction, then makes a small
                    // rotation towards the chosen direction so that if a robot of given dimensions
                    // (max dimensions) was in front the sensor would not be able to detect it any more.
                    // Checks if object is still in front. If it is, then move in the direction opposite
                    // to that chosen. If it isn’t move forward until it is safe to change direction
                    // back to the original objective.
                    moveState = (usmLevel < 30.48) ? 3 : 1;
                }
                if((usmLevel > 30.48 && aTimeStart < 2) || (usmLevel > 30.48 && aDistToTrav < aDistTrav)){
                    gameState = metaGameState;
                    metaGameState = -1;
                }
                aDistTrav += dDist * DEGREES_TO_FEET;
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
                //Case Three is independent turning. It cares not about our heading, but instead
                //uses aPrefDir to pick which way to turn. This is used in Case 10 exclusively atm.
                power = .25;
                if(aPrefDir){
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
            case 5:
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
        telemetry.addData("Ultrasonic: ", usmLevel);
        telemetry.addData("aTimeStart: ", aTimeStart);
        telemetry.addData("aDistToTrav ", aDistToTrav);
        telemetry.addData("aDistTrav ", aDistTrav);

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
