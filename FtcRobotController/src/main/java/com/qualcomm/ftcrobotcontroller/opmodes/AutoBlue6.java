package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Travis on 10/3/2015.
 * Team 10464 Autonomous program
 */

public class AutoBlue6 extends AutonomousBase {
    @Override
    public void gameState(){

        super.gameState();
        double lineUp= 3.8;
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
                map.setGoal(9.25,lineUp);
                linedUp(1,2);
                if(map.distanceToGoal()<=.1){
                    moveState = 0;
                    gameState = 3;
                }
                break;
            case 3: //move to climber deposit
                map.setGoal(11.5,lineUp);
                linedUp(1,2);
                if(map.distanceToGoal() <= .1) {
                    moveState = 0;
                    gameState = 4;
                }
                break;
            case 4: // line up, and drop climbers
                map.setGoal(12, lineUp);
                linedUp(5,2);
                if(climbTime > 0 && getRuntime() > climbTime+1){
                    moveState = 0;
                    gameState = 5;
                }
                break;
            case 5: // Back up to avoid wall while turning
                map.setGoal(11, lineUp);
                if (Math.abs(heading - map.angleToGoalRev()) < TOL || (heading > 360 - TOL && map.angleToGoalRev() < TOL || (heading < TOL && map.angleToGoalRev() > 360 - TOL))) {
                    moveState = 3;
                } else {
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
    }
}
