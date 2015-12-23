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
    DcMotor motorC;
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
    boolean aPrefDir; //Avoidance Preferred direction. True=left & False=right
    double minHead;
    double aX;
    double aY;
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

        motorLT.setDirection(DcMotor.Direction.FORWARD);
        motorLB.setDirection(DcMotor.Direction.FORWARD);

        motorRT.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);

        motorA = hardwareMap.dcMotor.get("motor_A");
        motorS = hardwareMap.dcMotor.get("motor_S");
        motorC = hardwareMap.dcMotor.get("motor_C");
        climber = hardwareMap.servo.get("climber");
        swingLeft = hardwareMap.servo.get("swing_l");
        swingRight = hardwareMap.servo.get("swing_r");

        climber.setPosition(1);
        swingLeft.setPosition(.8);
        swingRight.setPosition(.8);

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
            if(gameState<9) { //technically, gameState 10 can be here, and I want to reserve meta's integrity.
                metaGameState = gameState;
            }
            gameState = 8; //avoid
        }
    }
    public void linedUp(int o, int n) {
        if (Math.abs(heading - map.angleToGoal()) < TOL || (heading > 360 - TOL && map.angleToGoal() < TOL || (heading < TOL && map.angleToGoal() > 360 - TOL))) {
            moveState = o;
        } else {
            moveState = n;
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
                    gameState = 1;
                }
                break;
            case 1: //Move up before turning to beacon
                map.setGoal(6,9);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;  // stop the robot
                    gameState = 2;  // Move to the next stage.
                }
                break;
            case 2: //Move to beacon
                map.setGoal(9.25, 6);
                //Checks our heading.
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;  // stop the robot
                    gameState = 3;  // Move to the next stage.
                }
                aPrefDir = true; //left. We need to be extremely careful with crossing over midline.
                //if(map.distanceToGoal()>1.5) avoid();
                break;
            case 3: //move to climber deposit
                map.setGoal(10.25, 6);
                //Checks our heading.
                linedUp(1,2);
                if(map.distanceToGoal()<=.1 || usmLevel < 5) {
                    moveState = 0;  // stop the robot
                    gameState = 4;  // Move to the next stage.
                }
                break;
            case 4: // line up, and drop climbers
                map.setGoal(11,6);
                linedUp(5,2);
                if(Math.abs(climber.getPosition()-.25) < .02){
                    moveState = 0;
                    gameState = 5;
                }
                break;
            case 5: // move to ramp alignment spot
                map.setGoal(8, 8);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) { //TODO: '|| colorsensor = white'
                    moveState = 0;  // stop the robot
                    gameState = 6;  // Move to the next stage.
                }
                aPrefDir = false; //Right is better for us.
                //avoid(); //may act erratically since we start on the wall.
                break;
            case 6: //align with ramp, and gun it up.
                map.setGoal(53,45);
                linedUp(1,2);
                break;
            case 8: // Move Around.
                aTimeStart = sTime;
                aDistTrav = 0;
                gameState = 9;
            case 9:
                minHead = heading-map.angleToGoal();
                if(aTimeStart + 2 < sTime) {
                    moveState = 3;
                    if(usmLevel > 30){
                        aX = map.getRobotX();
                        aY = map.getRobotY();
                        gameState = 10;
                    }
                }else if(usmLevel > 30){
                    gameState = metaGameState;
                }
                break;
            case 10:
                map.setGoal(aX+Math.cos(Math.toRadians(minHead)),aY+Math.sin(Math.toRadians(minHead)));
                if(map.distanceToGoal() <= .1){
                    gameState = metaGameState;
                }
                //avoid(); //In case something is encountered on our new path, restart calculations.
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
                boolean turnRight;

                if(heading<=180){
                    turnRight = heading<=map.angleToGoal()&&heading+180>=map.angleToGoal();
                }

                else  {
                    turnRight = !                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              (heading>=map.angleToGoal()&&heading-180<=map.angleToGoal());
                }

                if (turnRight) {

                    motorRT.setPower(-power);
                    motorRB.setPower(-power);
                    motorLT.setPower(power);
                    motorLB.setPower(power);
                } else {
                    motorRT.setPower(power);
                    motorRB.setPower(power);
                    motorLT.setPower(-power);
                    motorLB.setPower(-power);
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
        telemetry.addData("Encoder Data :",(motorLB.getCurrentPosition()+motorRB.getCurrentPosition()+motorLT.getCurrentPosition()+motorRT.getCurrentPosition())/4);
        telemetry.addData("LMB :",motorLB.getCurrentPosition());
        telemetry.addData("RMB :",motorRB.getCurrentPosition());
        telemetry.addData("LMT :",motorLT.getCurrentPosition());
        telemetry.addData("Climber pos :",climber.getPosition());


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
