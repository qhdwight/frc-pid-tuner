package team8.tuner.controller;

import com.revrobotics.*;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import team8.tuner.config.Config.MasterConfig;
import team8.tuner.config.Config.SimpleConfig;

import java.util.Optional;
import java.util.function.Function;

import static team8.tuner.Robot.kPidSlotIndex;

public class Spark extends ControllerBase<CANSparkMax> {

    private final CANPIDController mPidController;
    private final CANEncoder mEncoder;

    public Spark(MasterConfig config) {
        this((SimpleConfig) config);
        configureSoftLimit(SoftLimitDirection.kForward, config.forwardLimit);
        configureSoftLimit(SoftLimitDirection.kReverse, config.reverseLimit);
        mController.setInverted(config.isInverted);
        check(mController.setIdleMode(config.isBraked ? IdleMode.kBrake : IdleMode.kCoast), "idle mode");
        check(mController.enableVoltageCompensation(config.voltageCompensation), "voltage compensation");
        check(mController.setClosedLoopRampRate(config.ramp), "closed loop ramp");
        check(mPidController.setP(config.gains.p, kPidSlotIndex), "p");
        check(mPidController.setI(config.gains.i, kPidSlotIndex), "i");
        check(mPidController.setD(config.gains.d, kPidSlotIndex), "d");
        check(mPidController.setFF(config.gains.f, kPidSlotIndex), "f");
        check(mPidController.setIMaxAccum(config.gains.iMax, kPidSlotIndex), "i max");
        check(mPidController.setIZone(config.gains.iZone, kPidSlotIndex), "i zone");
        check(mPidController.setOutputRange(config.minimumOutput, config.maximumOutput, kPidSlotIndex), "output range");
        check(mPidController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, kPidSlotIndex), "strategy");
        check(mPidController.setSmartMotionMaxVelocity(config.gains.v, kPidSlotIndex), "max velocity");
        check(mPidController.setSmartMotionMaxAccel(config.gains.a, kPidSlotIndex), "max acceleration");
        check(mPidController.setSmartMotionAllowedClosedLoopError(config.gains.allowableError, kPidSlotIndex), "allowable error");
        check(mPidController.setSmartMotionMinOutputVelocity(0.0, kPidSlotIndex), "min velocity");
        check(mEncoder.setPositionConversionFactor(config.positionConversion), "position conversion");
        check(mEncoder.setVelocityConversionFactor(config.velocityConversion), "velocity conversion");
        check(mEncoder.setPosition(config.startingPosition), "starting position");
    }

    public Spark(SimpleConfig config) {
        super(config.id);
        mPidController = mController.getPIDController();
        mEncoder = mController.getEncoder();
        System.out.printf("Setup spark with id %d%n", config.id);
        check(mController.restoreFactoryDefaults(), "factory defaults");
    }

    private void configureSoftLimit(SoftLimitDirection direction, Float configLimit) {
        var limit = Optional.ofNullable(configLimit);
        check(mController.enableSoftLimit(direction, limit.isPresent()), "enable soft limit");
        limit.ifPresent(softLimit -> check(mController.setSoftLimit(direction, softLimit), "set soft limit"));
    }

    private void check(CANError error, String name) {
        if (error != CANError.kOk) {
            var message = String.format("Failed to set %s! Error: %s", name, error);
            System.err.println(message);
            throw new RuntimeException(message);
        }
    }

    @Override
    Function<Integer, CANSparkMax> controllerFactory() {
        return deviceId -> new CANSparkMax(deviceId, MotorType.kBrushless);
    }

    @Override
    public void follow(Controller master) {
        try {
            var masterSpark = (Spark) master;
            mController.follow(masterSpark.mController);
        } catch (Exception exception) {
            exception.printStackTrace();
        }
    }

    @Override
    public void setOutput(ControlMode controlMode, double reference, double arbitraryFeedForward) {
        ControlType controlType;
        switch (controlMode) {
            case DISABLED:
                controlType = ControlType.kDutyCycle;
                reference = 0.0;
                break;
            case PERCENT_OUTPUT:
                controlType = ControlType.kDutyCycle;
                break;
            case SMART_MOTION:
                controlType = ControlType.kSmartMotion;
                break;
            case SMART_VELOCITY:
                controlType = ControlType.kSmartVelocity;
                break;
            default:
                throw new IllegalStateException("Unknown control mode!");
        }
        mPidController.setReference(reference, controlType, kPidSlotIndex, arbitraryFeedForward);
    }

    @Override
    public double getOutputCurrent() {
        return mController.getOutputCurrent();
    }

    @Override
    public double getPosition() {
        return mEncoder.getPosition();
    }

    @Override
    public double getVelocity() {
        return mEncoder.getVelocity();
    }

    @Override
    public double getAppliedOutput() {
        return mController.getAppliedOutput();
    }
}
