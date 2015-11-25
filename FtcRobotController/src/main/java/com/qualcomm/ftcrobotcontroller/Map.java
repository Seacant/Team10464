package com.qualcomm.ftcrobotcontroller;

import android.util.Log;

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

    public double getRobotX(){
        return robotX;
    }

    public double getRobotY(){
        return robotY;
    }

    public double angleToGoal(){
        double dX = goalX-robotX;
        double dY = goalY-robotY;
        return (((Math.atan2(dY, dX) * 180) / Math.PI) + 450) % 360; //Ask Travis why we +270%360
    }

    public double distanceToGoal(){
        double dX = goalX-robotX;
        double dY = goalY-robotY;
        return Math.sqrt(dX * dX + dY * dY); //return length of hypotenuse
    }

    public void moveRobot(double feet,double heading) {
        robotX -= feet * Math.cos(Math.toRadians((heading + 450) % 360));
        robotY -= feet * Math.sin(Math.toRadians((heading + 450) % 360));
    }
}
