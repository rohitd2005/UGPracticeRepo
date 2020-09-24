package org.firstinspires.ftc.teamcode.Resources;

public class CurvePoint {
    public double x;
    public double y;
    public double lookaheadDistance;
    public double distanceAlongPath = 0;
    public int pointIndex = 0;
    private double movementSpeed;
    private double turnSpeed;
    private double decelDistance;
    private double decelTurn;
    private double faceTowardsAngle;

    public CurvePoint(double x , double y){
        this.x = x;
        this.y = y;
    }

    public void setPoint(double x , double y){
        this.x = x;
        this.y = y;
    }

    public CurvePoint(double x , double y , int pointIndex){
        this.x = x;
        this.y = y;
        this.pointIndex = pointIndex;
    }

    public CurvePoint(Point point, double movementSpeed , double turnSpeed , double decelDistance , double decelTurn , double faceTowardsAngle , double lookaheadDistance){
        this.x = point.x;
        this.y = point.y;
        this.movementSpeed = movementSpeed;
        this.turnSpeed = turnSpeed;
        this.decelDistance = decelDistance;
        this.decelTurn = decelTurn;
        this.faceTowardsAngle = faceTowardsAngle;
        this.lookaheadDistance = lookaheadDistance;
    }

    public CurvePoint(){

    }

    public void setX(double x){
        this.x = x;
    }

    public void setY(){
        this.y = y;
    }

    public void setLookaheadDistance(double lookaheadDistance){
        this.lookaheadDistance = lookaheadDistance;
    }

    public void setMovementSpeed(double movementSpeed){
        this.movementSpeed = movementSpeed;
    }

    public void setDecelDistance(double decelDistance){
        this.decelDistance = decelDistance;
    }

    public void setDecelTurn(double decelTurn){
        this.decelTurn = decelTurn;
    }

    public void setFaceTowardsAngle(double faceTowardsAngle){
        this.faceTowardsAngle = faceTowardsAngle;
    }

    public void setDistanceAlongPath(double dist){
        this.distanceAlongPath = dist;
    }

    public double getDistanceAlongPath(){
        return distanceAlongPath;
    }

    public Vector toVector(){
        return new Vector(this.x , this.y);
    }

    public double getDecelDistance(){
        return decelDistance;
    }

    public double getTurnSpeed(){
        return turnSpeed;
    }

    public double getDecelTurn(){
        return decelTurn;
    }

    public double getMovementSpeed(){
        return movementSpeed;
    }

    public double getFaceTowardsAngle(){
        return faceTowardsAngle;
    }

    public void setTurnSpeed(double turnSpeed){
        this.turnSpeed = turnSpeed;
    }

}
