package com.qualcomm.ftcrobotcontroller;

/**
 * Controls all map logic for autonomous
 */

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

public class Map {
    double goalX, goalY;
    double robotX, robotY;
    String teamColor;

    public Map(String teamColor){ //pass in Team color
        this.teamColor = teamColor;
        if(teamColor.equals("Blue")){
            robotX = 6;
            robotY = 11;
        }else{
            robotX = 6;
            robotY = 0;
        }
    }
    public void setGoal(int x, int y){
        goalX = x;
        goalY = y;
    }
    public double getGoalX(){
        return goalX;
    }
    public double getGoalY(){
        return goalY;
    }
    public double angleToGoal(){
        double dX = robotX-goalX;
        double dY = robotY-goalY;
        return (Math.tan(dY/dX)*180)/Math.PI; //return degrees because that's what the gyro uses.
    }
}
