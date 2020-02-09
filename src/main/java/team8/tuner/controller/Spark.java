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

	public Spark(SimpleConfig config) {
		super(config.id);
		mPidController = mController.getPIDController();
		mEncoder = mController.getEncoder();
		check(mController.restoreFactoryDefaults(), "factory defaults");
		check(mController.setIdleMode(config.isBraked ? IdleMode.kBrake : IdleMode.kCoast), "idle mode");
		if (config instanceof MasterConfig) {
			var masterConfig = (MasterConfig) config;
			configureSoftLimit(SoftLimitDirection.kForward, masterConfig.forwardLimit);
			configureSoftLimit(SoftLimitDirection.kReverse, masterConfig.reverseLimit);
			mController.setInverted(masterConfig.isInverted);
			check(mController.enableVoltageCompensation(masterConfig.voltageCompensation), "voltage compensation");
			check(mController.setOpenLoopRampRate(masterConfig.ramp), "open loop ramp");
			check(mController.setClosedLoopRampRate(masterConfig.ramp), "closed loop ramp");
			check(mPidController.setP(masterConfig.gains.p, kPidSlotIndex), "p");
			check(mPidController.setI(masterConfig.gains.i, kPidSlotIndex), "i");
			check(mPidController.setD(masterConfig.gains.d, kPidSlotIndex), "d");
			check(mPidController.setFF(masterConfig.gains.f, kPidSlotIndex), "f");
			check(mPidController.setIMaxAccum(masterConfig.gains.iMax, kPidSlotIndex), "i max");
			check(mPidController.setIZone(masterConfig.gains.iZone, kPidSlotIndex), "i zone");
			check(mPidController.setOutputRange(masterConfig.minimumOutput, masterConfig.maximumOutput, kPidSlotIndex), "output range");
			check(mPidController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, kPidSlotIndex), "strategy");
			check(mPidController.setSmartMotionMaxVelocity(masterConfig.gains.v, kPidSlotIndex), "max velocity");
			check(mPidController.setSmartMotionMaxAccel(masterConfig.gains.a, kPidSlotIndex), "max acceleration");
			check(mPidController.setSmartMotionAllowedClosedLoopError(masterConfig.gains.allowableError, kPidSlotIndex), "allowable error");
			check(mPidController.setSmartMotionMinOutputVelocity(0.0, kPidSlotIndex), "min velocity");
			check(mEncoder.setPositionConversionFactor(masterConfig.positionConversion), "position conversion");
			check(mEncoder.setVelocityConversionFactor(masterConfig.velocityConversion), "velocity conversion");
			check(mEncoder.setPosition(masterConfig.startingPosition), "starting position");
		}
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
	public void follow(Controller master, boolean isInverted) {
		try {
			var masterSpark = (Spark) master;
			mController.follow(masterSpark.mController, isInverted);
		} catch (Exception exception) {
			throw new RuntimeException("Could not follow!", exception);
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
	public double getAppliedPercentOutput() {
		return mController.getAppliedOutput();
	}
}
