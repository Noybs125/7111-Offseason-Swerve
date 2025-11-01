package team7111.robot.utils.swervemodules;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team7111.robot.Constants;
import team7111.robot.Constants.SwerveConstants;
import team7111.robot.utils.config.SwerveModuleConfig;
import team7111.robot.utils.encoder.GenericEncoder;

public class TalonFXSwerveModule implements GenericSwerveModule{

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private GenericEncoder encoder;

    private TalonFXConfiguration driveConfig;
    private TalonFXConfiguration angleConfig;

    private DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private PositionVoltage anglePosition = new PositionVoltage(0);

    private double encoderOffsetDegrees;

    public TalonFXSwerveModule(SwerveModuleConfig config){
        driveMotor = new TalonFX(config.driveMotor.id, Constants.canbus);
        angleMotor = new TalonFX(config.angleMotor.id, Constants.canbus);
        encoder = config.encoder;

        driveConfig = config.driveMotor.getTalonFXConfiguration();
        angleConfig = config.angleMotor.getTalonFXConfiguration();

        encoderOffsetDegrees = config.canCoderOffsetDegrees;
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond / SwerveConstants.maxDriveVelocity;
        driveDutyCycle.Output = speed;
        driveMotor.setControl(driveDutyCycle);
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        var speed = state.speedMetersPerSecond;
        driveVelocity.Velocity = speed / SwerveConstants.driveRPMToMPS;
        driveVelocity.FeedForward = driveFF.calculate(speed);
        driveMotor.setControl(driveVelocity);
    }

    @Override
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.driveRPMToMPS;
    }

    @Override
    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * SwerveConstants.driveRotationsToMeters;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        anglePosition.Position = rotation.getRotations();
        angleMotor.setControl(anglePosition);
    }

    @Override
    public GenericEncoder getEncoder() {
        return encoder;
    }

    @Override
    public void zeroWheels() {
        angleMotor.setPosition(Units.degreesToRotations(encoder.getPosition().getDegrees() - encoderOffsetDegrees));
    }

    @Override
    public void configure() {
        driveMotor.getConfigurator().apply(driveConfig);
        angleMotor.getConfigurator().apply(angleConfig);
    }    
}
