package team8.tuner;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.module.jsonSchema.JsonSchema;
import com.fasterxml.jackson.module.jsonSchema.JsonSchemaGenerator;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import team8.tuner.config.C;
import team8.tuner.config.Config;
import team8.tuner.config.Config.SimpleConfig;
import team8.tuner.controller.Controller;
import team8.tuner.controller.Controller.ControlMode;
import team8.tuner.controller.Spark;
import team8.tuner.controller.Talon;
import team8.tuner.controller.Victor;
import team8.tuner.controller.*;
import team8.tuner.data.CSVWriter;
import team8.tuner.data.LiveGraph;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

	//========================================================//
	public static final String kConfigFileName = "Climber";
	//========================================================//

	public static final int kPidSlotIndex = 0;
	public static final double kPercentOutputMultiplier = 0.9, kVelocityMultiplier = 0.9;
	private static final double kDeadBand = 0.08;
	private Config mConfig;
	private Controller mMaster;
	private List<Controller> mSlaves;
	private List<Solenoid> mSolenoids;
	private XboxController mInput;
	private PowerDistributionPanel mPowerDistributionPanel;
	private double mReference;
	private boolean mRunningConstantPercentOutput;
	private boolean mExtendSolenoid, mEnableCompressor = true;
	private ControlMode mControlMode = ControlMode.DISABLED;
	private Compressor mCompressor = new Compressor();

	@Override
	public void robotInit() {
		mPowerDistributionPanel = new PowerDistributionPanel();
		if (RobotBase.isSimulation()) {
			var mapper = new ObjectMapper();
			var generator = new JsonSchemaGenerator(mapper);
			try {
				JsonSchema schema = generator.generateSchema(Config.class);
				mapper.writerWithDefaultPrettyPrinter().writeValue(new File("schema.json"), schema);
			} catch (IOException schemaGenerationException) {
				schemaGenerationException.printStackTrace();
			}
		}
	}

	@Override
	public void autonomousInit() {
		scoldUser();
	}

	@Override
	public void teleopInit() {
		scoldUser();
	}

	@Override
	public void testInit() {
		CSVWriter.init();
		mConfig = C.read(Config.class, kConfigFileName);
		applyConfig();
		LiveGraph.add("isEnabled", true);
	}

	private void applyConfig() {
		System.out.printf("Initializing PID tuner with:%n%s%n", mConfig);
		mInput = new XboxController(mConfig.xboxId);
		System.out.printf("Using X-Box controller with id: %d%n", mConfig.xboxId);
		mMaster = setupController(mConfig.master);
		mSlaves = mConfig.slaves.stream()
				.map(slaveConfig -> setupSlave(slaveConfig, mMaster))
				.collect(Collectors.toUnmodifiableList());
		mSolenoids = mConfig.solenoidId.stream().map(Solenoid::new).collect(Collectors.toUnmodifiableList());
	}

	@Override
	public void testPeriodic() {
		handleInput();
		periodicData();
		applyOutputs();
	}

	private void periodicData() {
		if (mConfig.writeCsv) {
			logData("totalPdpCurrent", mPowerDistributionPanel.getTotalCurrent());
			logData("totalControllerCurrent", mMaster.getOutputCurrent() + mSlaves.stream().mapToDouble(Controller::getOutputCurrent).sum());
			logData("reference", mReference);
			logData("output", mMaster.getAppliedPercentOutput());
			logData("position", mMaster.getPosition());
			logData("velocity", mMaster.getVelocity());
		}
	}

	private void logData(String name, double data) {
		CSVWriter.add(name, data);
		LiveGraph.add(name, data);
	}

	private void applyOutputs() {
		if (mMaster != null) {
			double arbitraryFeedForward;
			switch (mControlMode) {
				case PERCENT_OUTPUT:
				case SMART_MOTION:
				case SMART_VELOCITY:
					arbitraryFeedForward = mConfig.master.gains.ff;
					if (mConfig.master.armFf != null) {
						double angle = mMaster.getPosition();
						arbitraryFeedForward += mConfig.master.armFf * Math.cos(Math.toRadians(angle - mConfig.master.armComOffset));
					}
					break;
				default:
					arbitraryFeedForward = 0.0;
					break;
			}
			mMaster.setOutput(mControlMode, mReference, arbitraryFeedForward);
		}
		if (mSolenoids != null) {
			for (Solenoid solenoid : mSolenoids) {
				solenoid.set(mExtendSolenoid);
			}
		}
//		if (mEnableCompressor) {
//			mCompressor.start();
//		} else {
//			mCompressor.stop();
//		}
		mCompressor.stop();
	}

	@Override
	public void disabledInit() {
		mControlMode = ControlMode.DISABLED;
		mExtendSolenoid = false;
		mEnableCompressor = true;
		if (mSolenoids != null) {
			mSolenoids.forEach(Solenoid::close);
		}
		applyOutputs();
		if (mConfig != null && mConfig.writeCsv) CSVWriter.write();
		LiveGraph.add("isEnabled", false);
	}

	private void handleInput() {
		if (mInput.getAButtonPressed()) {
			setSetPoint(mConfig.aSetPoint);
		} else if (mInput.getBButtonPressed()) {
			setSetPoint(mConfig.bSetPoint);
		} else if (mInput.getXButtonPressed()) {
			setSetPoint(mConfig.xSetPoint);
		} else if (mInput.getYButtonPressed()) {
			setSetPoint(mConfig.ySetPoint);
		} else if (mInput.getBumperPressed(Hand.kRight)) {
			mControlMode = ControlMode.PERCENT_OUTPUT;
			mReference = mConfig.percentOutputRun + mConfig.master.gains.ff;
			mRunningConstantPercentOutput = true;
		} else if (mInput.getBumperPressed(Hand.kLeft)) {
			mControlMode = ControlMode.DISABLED;
			mRunningConstantPercentOutput = false;
			System.out.println("Disabling...");
		} else {
			double percentOutInput = -mInput.getY(Hand.kLeft) * kPercentOutputMultiplier;
			double velocityInput = -mInput.getY(Hand.kRight) * kVelocityMultiplier;
			if (Math.abs(percentOutInput) > kDeadBand) {
				mControlMode = ControlMode.PERCENT_OUTPUT;
				mReference = percentOutInput - Math.signum(percentOutInput) * kDeadBand;
				mRunningConstantPercentOutput = false;
			} else if (Math.abs(velocityInput) > kDeadBand) {
				mControlMode = ControlMode.SMART_VELOCITY;
				mReference = (velocityInput - Math.signum(velocityInput) * kDeadBand) * mConfig.master.gains.v;
				mRunningConstantPercentOutput = false;
			} else {
				if (!mRunningConstantPercentOutput) {
					mControlMode = ControlMode.DISABLED;
				}
			}
		}
		if (mInput.getStartButtonPressed())
			mExtendSolenoid = !mExtendSolenoid;
		else if (mInput.getBackButtonPressed())
			mEnableCompressor = !mEnableCompressor;
	}

	private void setSetPoint(double setPoint) {
		mControlMode = ControlMode.SMART_MOTION;
		mReference = setPoint;
	}

	private Controller setupController(SimpleConfig config) {
		System.out.printf("Setting up %s with id %d%n", config.type, config.id);
		switch (config.type) {
			case SPARK:
				return new Spark(config);
			case FALCON:
				return new Falcon(config);
			case TALON:
				return new Talon(config);
			case VICTOR:
				return new Victor(config);
			default:
				throw new IllegalArgumentException("Unknown motor type!");
		}
	}

	private Controller setupSlave(SimpleConfig config, Controller master) {
		var slave = setupController(config);
		slave.follow(master, config.isInverted);
		return slave;
	}

	private void scoldUser() {
		System.err.println("Use test mode!");
	}
}
