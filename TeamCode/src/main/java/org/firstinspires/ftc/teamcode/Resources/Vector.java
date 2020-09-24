package org.firstinspires.ftc.teamcode.Resources;

public class Vector {
    public double x,y,z;
    public double velocity, curvature, distance;

    public Vector(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
        velocity = 0;
        curvature = 0;
        distance = 0;
    }

    public Vector(double x, double y){
        this.x = x;
        this.y = y;
        velocity = 0;
        curvature = 0;
        distance = 0;
    }

    public Vector(Vector v){
        this.x = v.x;
        this.y = v.y;
        this.z = v.z;
        this.velocity = v.velocity;
        this.curvature = v.curvature;
        this.distance = v.distance;
    }

    public Vector(){}

    /**
     * Adds up two vectors
     *
     * @param a         The first vector
     * @param target    The target vector that's being added
     * @return  added vector
     */
    public Vector add(Vector a, Vector target){
        if (target == null) {
            target = new Vector(this.x + a.x , this.y + a.y);
        } else {
            target.set(this.x + a.x , this.y + a.y,0);
        }
        return target;
    }

    /**
     * Returns the length of the vector
     * @return
     */
    public double norm(){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    /**
     * Normalizes the target vector
     *
     * @param target    target vector
     * @return  normalized vector
     */
    public Vector normalize(Vector target) {
        if (target == null) target = new Vector();
        double m = norm();
        if (m > 0) target.set(x / m, y / m, z / m);
        else target.set(x, y, z);
        return target;
    }


    /**
     * Calculates magnitude of the vector
     * @return  magnitude (length)
     */
    public double getMagnitude(){return Math.hypot(x,y);}

    /**
     * Calculates the dot product of the two vectors
     *
     * @param vector    2nd vector
     * @return  dot product of the 2 vectors
     */
    public double dot(Vector vector){
        return (this.x * vector.x) + (this.y * vector.y);
    }

    /**
     * Multiplies the current vector by a scalar (factor)
     *
     * @param scalar    factor or multiplier
     * @return  scalared vector
     */
    public Vector multiply(double scalar){
        return new Vector(this.x * scalar,this.y * scalar);
    }

    public void mult(double scalar){
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
    }

    public static Vector mult(Vector a, double n, Vector target) {
        if (target == null) {
            target = new Vector(a.x * n, a.y * n, a.z * n);
        } else {
            target.set(a.x * n, a.y * n, a.z * n);
        }
        return target;
    }

    public static Vector add(Vector a, Vector b, Vector target) {
        if (target == null) {
            target = new Vector(a.x + b.x, a.y + b.y, a.z + b.z);
        } else {
            target.set(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        return target;
    }

    public static Vector sub(Vector a, Vector b){
        return new Vector(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    public void div(Vector a){
        x /= a.x;
        y /= a.y;
        z = a.z;
    }

    public static Vector multiply(double scalar, Vector v) {
        return new Vector(v.x * scalar, v.y * scalar);
    }

    public double getDistance(CurvePoint curvepoint){
        return curvepoint.distanceAlongPath;
    }

    public Vector displacement(Vector vector){
        return new Vector(vector.x - this.x,vector.y - this.y);
    }
/*
    public double distanceToVector(Vector vector){
        return Vector.calculateDistance(this.x,this.y,vector.x,vector.y);
    }*/

    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public boolean equals(Vector v){
        return (this.x == v.x) && (this.y == v.y);
    }

    public double getDirection(){return Math.toDegrees(Math.atan(y/x));}

    public CurvePoint toCurvePoint(){return new CurvePoint(x,y);}

    public void setDistance(double distance){
        this.distance = distance;
    }
}
