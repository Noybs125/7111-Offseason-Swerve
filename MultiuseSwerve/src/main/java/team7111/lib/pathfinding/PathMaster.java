package team7111.lib.pathfinding;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PathMaster {
    private PIDController xPID;
    private PIDController yPID;
    private PIDController rotPID;

    private Supplier<Pose2d> suppliedPose;
    private Supplier<Rotation2d> gyroYaw;

    private double xCalculation;
    private double yCalculation;
    private double rotCalculation;

    private double invertedX = 1.0;
    private double invertedY = 1.0;
    private double invertedRot = 1.0;
    private double invertedGyro = 1.0;

    //Houses pointers to field elements
    private FieldElement fieldElements[];

    private boolean fieldFlipped = false;
    private boolean fieldRelative = false;
    
    public PathMaster(Supplier<Pose2d> suppliedPose, Supplier<Rotation2d> gyroYaw){

        xPID = new PIDController(1, 0, 0);
        yPID = new PIDController(1, 0, 0);

        this.rotPID = new PIDController(1, 0, 0);
        rotPID.enableContinuousInput(-180, 180);
        this.suppliedPose = suppliedPose;
        this.gyroYaw = () -> Rotation2d.fromDegrees(gyroYaw.get().getDegrees() * invertedGyro);
        //this.gyroYaw = gyroYaw;
    }
    
    public void useAllianceFlipping(boolean flipField){
        fieldFlipped = flipField;
    }
    
    public void useFieldRelative(boolean isFieldRelative){
        fieldRelative = isFieldRelative;
    }

    public void setTranslationPID(double P, double I, double D){
        xPID.setPID(P, I, D);
        yPID.setPID(P, I, D);
    }

    public void setRotationPID(double P, double I, double D){
        rotPID.setPID(P, I, D);
    }

    public void setFieldElementMap(FieldElement[] fieldElementArray){
        fieldElements = fieldElementArray;
    }

    public void setInversions(boolean invertX, boolean invertY, boolean invertRot, boolean invertGyro){
        invertedX = 1.0;
        invertedY = 1.0;
        invertedRot = 1.0;
        invertedGyro = 1.0;

        if (invertX){
            invertedX = -1.0;
        }

        if (invertY){
            invertedY = -1.0;
        }

        if (invertRot){
            invertedRot = -1.0;
        }

        if (invertGyro){
            invertedGyro = -1.0;
        }
    }

    /**
     * Flips path waypoint's positions and rotations. Keeps origin.
     */
    public void flipField(Path path){
        path.flipPath();
    }
    
    public void initializePath(Path path){
        path.setPoseSupplier(suppliedPose);
        path.setSpeedSuppliers(()-> xCalculation, ()-> yCalculation, ()-> rotCalculation);
        path.initialize();
    }

    public void periodic(Path path){
        path.periodic();
        xCalculation = xPID.calculate(suppliedPose.get().getX(), path.getCurrentWaypoint().getPose().getX()) * invertedX;
        yCalculation = yPID.calculate(suppliedPose.get().getY(), path.getCurrentWaypoint().getPose().getY()) * invertedY;
        rotCalculation = rotPID.calculate(suppliedPose.get().getRotation().getDegrees(), path.getCurrentWaypoint().getPose().getRotation().getDegrees()) * invertedRot;

    }

    public ChassisSpeeds getPathSpeeds(Path path, boolean avoidFieldElements, boolean fieldRelative){
        
        ChassisSpeeds chassisSpeeds = fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(path.getTranslationXSpeed(), path.getTranslationYSpeed(), path.getRotationSpeed(), gyroYaw.get())
            : new ChassisSpeeds(path.getTranslationXSpeed(), path.getTranslationYSpeed(), path.getRotationSpeed());

        return chassisSpeeds;

    }
}
