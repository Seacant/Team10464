package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Travis on 10/3/2015.
 * Team 10464 Autonomous program
 */

public class AutoBlue62 extends AutonomousBase {
    @Override
    public void gameState() {

        super.gameState();

        //Goal-specific logic
        switch(gameState){
            case 0: //Start of game:
                if(tDiff == 0){tDiff = getRuntime();}
                if(getRuntime() > 5 || !gyro.isCalibrating()) {
                    gameState = 1;
                }
                break;
            case 1: //Move up
                map.setGoal(startPos,9.5);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;
                    gameState = 2;
                }
                break;
            case 2: //Sweep
                map.setGoal(10, 6.5);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;
                    gameState = 3;
                }
                break;
            case 3: //move to red ramp
                map.setGoal(7, 8.5);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1) {
                    moveState = 0;
                    gameState = 4;
                }
                break;
            case 4: //align with ramp, and gun it up.
                map.setGoal(47, 45);
                linedUp(1,2);
                break;
            case 777:
                moveState = 0;
                break;
        }
    }
}
