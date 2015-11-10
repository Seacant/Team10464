package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Travis on 10/3/2015.
 * Team 10464 Autonomous program
 */
public class Autonomous extends OpMode {

    DcMotor motorRT;
    DcMotor motorRB;
    DcMotor motorLT;
    DcMotor motorLB;
    DcMotor motorA;

    Servo servoArm;

    //We stateful now, boys.
    int sensorState = 0;
    int gameState = 0;
    int moveState = 0;

    /**
     * Constructor
     */
    public Autonomous() {

    }

    /*
     * Code to run when the op mode is first enabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init() {
        motorRT = hardwareMap.dcMotor.get("motor_RT");
        motorRB = hardwareMap.dcMotor.get("motor_RT");

        motorLT = hardwareMap.dcMotor.get("motor_LT");
        motorLB = hardwareMap.dcMotor.get("motor_LT");
        motorLT.setDirection(DcMotor.Direction.REVERSE);
        motorLB.setDirection(DcMotor.Direction.REVERSE);

        motorA = hardwareMap.dcMotor.get("motor_arm");
        servoArm = hardwareMap.servo.get("servo_arm");

        //The information below will henceforth be referred to as the 'brain' of this program.
        String Team = "Red";
        //We need to, in memory, have a map detailing what is where on the field. I believe a 2d
        //array will be the best way of handling this, as it allows us to not only store where we
        //are, but where various constant pieces of the field are at (collection zones, mountains,
        //etc.
        int[][] map = { //TODO: Break map up into recognizable chunks for position error checking
                        //TODO: and intelligent decision making
                        /*BlueMountain*/
                {0,0,0,0,0,0,0,0,3,3,3,3}, /*Red Side*/
                {0,0,0,0,0,0,0,0,0,3,3,3},
                {0,0,0,0,0,0,0,0,0,0,3,3},
                {0,0,0,0,0,0,0,0,0,0,0,3},/*RedMountain*/
                {0,0,0,0,0,0,0,0,0,0,0,0},
                {0,0,0,0,0,0,0,0,0,0,0,0},
                {0,0,0,0,0,0,0,0,0,0,0,0},
                {3,0,0,0,0,0,0,0,0,0,0,0},
                {3,3,0,0,0,0,0,0,0,0,0,0},
/*RedMountain*/ {3,3,3,0,0,0,0,0,0,0,0,0},
                {3,3,3,3,0,0,0,0,0,0,0,0}
 /*Blue Side*/  /*BlueMountain*/
        }; //12x12 2d array; reference by saying map[x][y];
        //We divide the map into 144 1ft x 1ft squares to simplify calculations.

        //LEGEND
        //0 = blank-ish space
        //1 = our robot
        //2 = suspected robots
        //3 = mountain
        //... TODO: finish this
        //9 = goal (Where we should be moving to) (NOT SET IN INIT)

    }

    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {
        //Information gathering phase
        switch(sensorState){
            case 0: //Start:
                //Deciding what goes in here is difficult- How much do we really need to know to
                //accurately start driving our robot? This being said, I am having an internal
                //argument as to making this a 'default' class, where if we even don't know what we
                //need, we can set sensorState to 0 and hope for the best.
                break;
            case 1: //Color sensor logic
                //
                break;
            case 2:
                //I'm now having the internal debate as to the importance of sensor states... It's
                //not like there will ever be an instance where we want to ignore data, right? We
                //could always treat the sensor case as a 'how do we interpret this data', due to
                //our absence of a case for that, but I cannot help feeling like that encroaches on
                //being TOO modular.
                break;
        } //It is expected that, by now, all sensory information is committed to 'memory', and ready
          //to be digested by the rest of the program (hence being at the beginning of the loop).


        //Goal-specific logic
        switch(gameState){
            case 0: //Start of game:
                //It was recommended to us that we should wait at the gate for a few second to allow
                //our teammate to GTFO, avoiding unnecessary beginning-game collisions.
                if(getRuntime() > 5) {
                    gameState = 1;
                }
                break;
            case 1: //Colour sensor
                //I'm sure there is more we can do here, I just don't know what.
                //// TODO: 10/27/2015 Expand on gameState 1 uses
                sensorState = 1;
        }
        switch(moveState){
            //Never should we be just 'moving', always move TOWARDS something.

        }


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
