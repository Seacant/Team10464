package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.ftcrobotcontroller.Map;

/**
 * Created by Travis on 10/3/2015.
 * Team 10464 Autonomous program
 */

public class Autonomous extends OpMode {
    public final double TOL = .1; //tolerance for heading calculations
    DcMotor motorRT;
    DcMotor motorRB;
    DcMotor motorLT;
    DcMotor motorLB;
    DcMotor motorA;
    DcMotor motorS;
    GyroSensor gyro;

    //We stateful now, boys.
    int sensorState;
    int gameState;
    int moveState;

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
    int xVal,yVal,zVal,heading;

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
        motorRB = hardwareMap.dcMotor.get("motor_RT");

        motorLT = hardwareMap.dcMotor.get("motor_LT");
        motorLB = hardwareMap.dcMotor.get("motor_LT");

        motorLT.setDirection(DcMotor.Direction.REVERSE);
        motorRB.setDirection(DcMotor.Direction.REVERSE);

        motorA = hardwareMap.dcMotor.get("motor_A");
        motorS = hardwareMap.dcMotor.get("motor_S");

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
        switch(sensorState){
            case 0: //Start:
                //Deciding what goes in here is difficult- How much do we really need to know to
                //accurately start driving our robot? This being said, I am having an internal
                //argument as to making this a 'default' class, where if we even don't know what we
                //need, we can set sensorState to 0 and hope for the best.
                break;
            case 1: //Color sensor logic
                heading = gyro.getHeading();
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
                //our teammate to GTFO, avoiding unnecessary beginning-game collisions. It will also
                //give our gyro a second to calibrate.
                if(getRuntime() > 5 && !gyro.isCalibrating()) {
                    gameState = 1;
                }
                break;
            case 1: //Colour sensor
                //// TODO: 10/27/2015 Expand on gameState 1 uses
                map.setGoal(0,7);
                sensorState = 1;
        }
        switch(moveState){
            //Never should we be just 'moving', always move TOWARDS something.
            case 1:
                //Case one is 'move towards' in the most literal sense. It assumes the path is
                //clear, and that there is a goal(9), and us(1) on the map somewhere.

                //Checks our heading.
                if(Math.abs(heading-map.angleToGoal()) < TOL){

                }else{
                    moveState = 2;
                }
            case 2:
                //Case Two is 'turn towards'
            case 3:
                //Case Three is 'move around'. implies there is something in front of us that we'd
                //like to not hit.
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
