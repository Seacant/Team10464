package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Jonathan on 2/13/2016.
 */
public class AutoBlue32 extends AutonomousBase {
    @Override
    public void gameState(){

        super.gameState();
        double lineUp= 5.5;
        startPos = 3;
        //Goal-specific logic
        switch(gameState){
            case 0: //Start of game:
                if(tDiff == 0){tDiff = getRuntime();}
                if(getRuntime() > 15 && !gyro.isCalibrating()) {
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
                linedUp(6,2);
                if(touch.isPressed()){
                    moveState = 5;
                    gameState = 777;
                }
                if(map.distanceToGoal() <= .1) {
                    moveState = 0;
                    gameState = 777;
                }
                break;
            case 777:
                moveState = 0;
                break;
        }
    }
}
