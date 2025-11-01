package team7111.lib.pathfinding;


/**
 * Class to hold waypoint's constraints. Contains a max speed, a min speed, and a tolerance.
 */
public class WaypointConstraints
{
    private double maxSpeed;
    private double minSpeed;
    /**In meters*/
    private double tolerance;

    /**
     * Constructs the waypoint parameter object. Sets just translation values.
     * <p> Defaults rotation parameters to: Rotation tolerance is 360 (full circle), and max rotation speed is 1rpm.
     * @param maxSpeed -Maximum translation speed in meters per second.
     * @param minSpeed -Minimum translation speed the robot must be going during both travel and end of waypoint.
     * @param tolerance -Translation tolerance to the waypoint the robot must be to be considered done.
     */
    public WaypointConstraints(double maxSpeed, double minSpeed, double tolerance)
    {
        this.maxSpeed = maxSpeed;
        this.minSpeed = minSpeed;
        this.tolerance = tolerance;
    }

    /**
     * Returns the rotation tolerance the robot needs to be to the waypoint in digrees.
     */
    public double getTolerance()
    {
        return this.tolerance;
    }


    /**
     * Returns maximum allowed translation speed in meters per second.
     */
    public double getMaxSpeed()
    {
        return this.maxSpeed;
    }

    /**
     * Returns minimum allowed translation speed in meters per second.
     */
    public double getMinSpeed()
    {
        return this.minSpeed;
    }

};