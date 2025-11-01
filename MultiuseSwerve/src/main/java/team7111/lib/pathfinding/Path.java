package team7111.lib.pathfinding;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Path {

    private double mapLengthX = 0;
    private double mapLengthY = 0;

    private boolean isflipped = false;

    //Robot position as a supplied Pose2d
    private Supplier<Pose2d> robotPose;

    //Translation speed of the robot in each axis, and rotation.
    private DoubleSupplier xTransSpeed = () -> 0.0;
    private DoubleSupplier yTransSpeed = () -> 0.0;
    private DoubleSupplier rotTransSpeed = () -> 0.0;

    private int currentWaypointIndex = 0;
    
    private Waypoint[] waypoints;
    private boolean isPathFinished = false;

    /**
     * Constructs a path from several waypoints. Uses pathMaster class to define parameters.
     * @param waypoints -An array of waypoint objects.
     * @param robotPose -A supplier to be assigned to the local variable, letting path know robot pose.
     */
    public Path(Waypoint[] waypoints)
    {
        this.waypoints = waypoints;

        var field = new Field2d();
        int index = 0;
        for(Waypoint waypoint : waypoints){
            field.getObject("Waypoint " + index).setPose(waypoint.getPose());
            index ++;
        }
        SmartDashboard.putData("Path Waypoints", field);
    }

    /**
     * Returns the current waypoint the robot is pathing to.
     * <p>Note current waypoint index is one ahead of array for waypoint.
     */
    public int getCurrentWaypointIndex()
    {
        return currentWaypointIndex;
    }

    /**
     * Returns the current waypoint object that the robot is pathing to.
     */
    public Waypoint getCurrentWaypoint()
    {
        return waypoints[currentWaypointIndex];
    }

    public Waypoint[] getWaypoints(){
        return waypoints;
    }
    
    /**
     * Returns in digrees per second, positive clockwise.
     */
    public double getRotationSpeed()
    {
        double speed = rotTransSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getRotationConstraints();
        if(speed > waypointConstraints.getMaxSpeed()) {
            speed = waypointConstraints.getMaxSpeed();
        }
        if(speed < -waypointConstraints.getMaxSpeed()){
            speed = -waypointConstraints.getMaxSpeed();
        }
        if(Math.abs(speed) < waypointConstraints.getMinSpeed()){
            if(speed > 0){
                speed = waypointConstraints.getMinSpeed();
            }else{
                speed = -waypointConstraints.getMinSpeed();
            }
        }
        //speed = rotTransSpeed.getAsDouble();
        return speed;
    }

    /**
     * Returns in meters per second.
     */
    public double getTranslationXSpeed()
    {
        double speed = xTransSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getTranslationConstraints();
        if(speed > waypointConstraints.getMaxSpeed()) {
            speed = waypointConstraints.getMaxSpeed();
        }
        if(speed < -waypointConstraints.getMaxSpeed()){
            speed = -waypointConstraints.getMaxSpeed();
        }
        if(Math.abs(getCurrentWaypoint().getPose().minus(robotPose.get()).getX())
         < Math.abs(getCurrentWaypoint().getPose().minus(robotPose.get()).getY())){
            if(Math.abs(speed) < waypointConstraints.getMinSpeed()){
                if(speed > 0){
                    speed = waypointConstraints.getMinSpeed();
                }else{
                    speed = -waypointConstraints.getMinSpeed();
                }
            }
        }
        //speed = xTransSpeed.getAsDouble();
        return speed;
    }

    /**
     * Returns in meters per second.
     */
    public double getTranslationYSpeed()
    {
        double speed = yTransSpeed.getAsDouble();
        WaypointConstraints waypointConstraints = getCurrentWaypoint().getTranslationConstraints();
        if(speed > waypointConstraints.getMaxSpeed()) {
            speed = waypointConstraints.getMaxSpeed();
        }
        if(speed < -waypointConstraints.getMaxSpeed()){
            speed = -waypointConstraints.getMaxSpeed();
        }
        if(Math.abs(getCurrentWaypoint().getPose().minus(robotPose.get()).getX())
         > Math.abs(getCurrentWaypoint().getPose().minus(robotPose.get()).getY())){
            if(Math.abs(speed) < waypointConstraints.getMinSpeed()){
                if(speed > 0){
                    speed = waypointConstraints.getMinSpeed();
                }else{
                    speed = -waypointConstraints.getMinSpeed();
                }
            }
        }
        return speed;
    }

    /**
     * Sets the suppliers for speed on the robot to be equal to local variables.
     * @param xSpeed -X axis speed in meters per second.
     * @param ySpeed -Y axis speed in meters per second.
     * @param rotSpeed -Rotation speed in digrees per second.
     */
    public void setSpeedSuppliers(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rotSpeed)
    {
        xTransSpeed = xSpeed;
        yTransSpeed = ySpeed;
        rotTransSpeed = rotSpeed;
    }

    /**
     * Sets the path robot position equal to an outside pose supplier.
     */
    public void setPoseSupplier(Supplier<Pose2d> pose){
        robotPose = pose;
    }
    
    /**
     * Returns if the path is finished or not
     */
    public boolean isPathFinished()
    {
        return isPathFinished;
    }

    /**
     * Resets key variables
     */
    public void initialize()
    {
        currentWaypointIndex = 0;
        isPathFinished = false;
    }

    /**
     * Flips the waypoint positions and rotations of all waypoints
     */
    public void flipPath()
    {
        //X and y from origins.
        double length = mapLengthX;
        double width = mapLengthY;
        for(int i = 0; i < waypoints.length; i++) {
            //Gets waypoint position
            double waypointX = waypoints[i].getPose().getX();
            double waypointY = waypoints[i].getPose().getY();
            double waypointRot = waypoints[i].getPose().getRotation().getDegrees();

            double newWayX;
            double newWayY;
            double newWayRot;
            //if true, then it needs to be blue alliance aligned again.
            if(isflipped){
                //Undoes flip
                newWayX = -waypointX + length;
                newWayY = -waypointY + width;
                isflipped = false;
            }else{
                //Flips
                newWayX = length - waypointX;
                newWayY = width - waypointY;
                isflipped = true;
            }
            //Rotation flipping
            newWayRot = waypointRot + 180;
            newWayRot = newWayRot % 360; //Sets angle to within 360.

            Waypoint newWaypoint = new Waypoint(new Pose2d(newWayX, newWayY, new Rotation2d()), 
            waypoints[i].getTranslationConstraints(), waypoints[i].getRotationConstraints());

            waypoints[i] = newWaypoint;
        }

    }
    
    /**
     * indexes waypoint to path to if there. 
     * If path is finished, sets path to finished and will not path to new waypoint.
     */
    public void periodic()
    {
        if(robotPose == null){
            System.out.println("RobotPose null");
            return;
        }
        if(waypoints == null){
            System.out.println("waypoints null");
            return;
        }
        if(waypoints[currentWaypointIndex].isAtWaypoint(robotPose.get()))
        {
            System.out.println("Next Waypoint");
            if(currentWaypointIndex == waypoints.length - 1){
                isPathFinished = true;
                System.out.println("Path Finished");
                return;
            }
            currentWaypointIndex++;
        }
    }
}
