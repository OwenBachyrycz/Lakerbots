package org.firstinspires.ftc.teamcode;

/**
 *
 * This class is used to set the correct motor power for each
 * motor given x, y, and rotational motion for a mecanum drivetrain.
 *
 */

public class MecanumMovement {

    private double y;
    private double x;
    private double rx;

    public MecanumMovement(double y, double x, double rx){
        this.y = y;
        this.x = x;
        this.rx = rx;
    }

    /**
     * Use this setter method for a new movement
     * @param y
     * @param x
     * @param rx
     */
    public void setMovement(double y, double x, double rx) {
        this.y = y;
        this.x = x;
        this.rx = rx;
    }

    //Allows functions in autonomous opmode access to the power to be set to each motor

    public double getFrontLeft(){
        return (y + x + rx);
    }
    public double getRearLeft(){
        return (y - x + rx);
    }
    public double getFrontRight(){
        return (y - x - rx);
    }
    public double getRearRight(){
        return (y + x -rx);
    }
}
