package org.firstinspires.ftc.teamcode.dependencies;

public class PositionVector {
    private double xpos;
    private double ypos;
    private double heading;
    public PositionVector(double xpos, double ypos, double heading){
        this.xpos = xpos;
        this.ypos = ypos;
        this.heading = heading;
    }

    public double getXPOS() {
        return xpos;
    }
    public double getYPOS(){
        return ypos;
    }
    public double getHeading(){
        return heading;
    }
    public void resetPosition(PositionVector newPosition){
        this.xpos = newPosition.getXPOS();
        this.ypos = newPosition.getYPOS();
        this.heading = newPosition.getHeading();
    }
    public void updateHeading(double newHeading){
        this.heading = newHeading;
    }
    public void updateXPOS(double newXPOS){
        this.xpos = newXPOS;
    }
    public void updateYPOS(double newYPOS){
        this.xpos = newYPOS;
    }
}
