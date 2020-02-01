package team8.tuner;

import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import team8.tuner.config.C;
import team8.tuner.config.Config;
import team8.tuner.config.Config.SimpleConfig;
import team8.tuner.controller.*;
import team8.tuner.controller.Controller.ControlMode;
import team8.tuner.csv.CSVWriter;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

    public static final int kPidSlotIndex = 0;
    public static final double kPercentOutputMultiplier = 0.4, kVelocityMultiplier = 0.7, kPercentOutputRun = 0.2;
    private static final double kDeadBand = 0.08;
    private Config mConfig;
    private Controller mMaster;
    private List<Controller> mSlaves;
    private List<Solenoid> mSolenoids;
    private XboxController mInput;
    private PowerDistributionPanel mPowerDistributionPanel;
    private double mReference;
    private boolean mRunningConstantPercentOutput;
    private boolean mExtendSolenoid;
    private ControlMode mControlMode = ControlMode.DISABLED;

    @Override
    public void robotInit() {
        mPowerDistributionPanel = new PowerDistributionPanel(0);
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
        mConfig = C.read(Config.class);
        applyConfig();
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
        periodicCsvData();
        applyOutputs();
    }

    private void periodicCsvData() {
        if (mConfig.writeCsv) {
            CSVWriter.addData("totalCurrent", mPowerDistributionPanel.getTotalCurrent());
            CSVWriter.addData("controllerCurrent", mMaster.getOutputCurrent() + mSlaves.stream().mapToDouble(Controller::getOutputCurrent).sum());
            CSVWriter.addData("output", mMaster.getAppliedOutput());
            CSVWriter.addData("position", mMaster.getPosition());
            CSVWriter.addData("velocity", mMaster.getVelocity());
        }
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
    }

    @Override
    public void disabledInit() {
        mControlMode = ControlMode.DISABLED;
        mExtendSolenoid = false;
        applyOutputs();
        if (mConfig != null && mConfig.writeCsv) CSVWriter.write();
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
            mReference = kPercentOutputRun + mConfig.master.gains.ff;
            mRunningConstantPercentOutput = true;
        } else if (mInput.getBumperPressed(Hand.kLeft)) {
            mControlMode = ControlMode.DISABLED;
            System.out.println("Disabling...");
        } else {
            double percentOutInput = -mInput.getY(Hand.kLeft) * kPercentOutputMultiplier;
            if (Math.abs(percentOutInput) > kDeadBand) {
                mControlMode = ControlMode.PERCENT_OUTPUT;
                mReference = percentOutInput - Math.signum(percentOutInput) * kDeadBand;
                mRunningConstantPercentOutput = false;
            } else {
                if (!mRunningConstantPercentOutput) mReference = 0.0;
            }
            double velocityInput = -mInput.getY(Hand.kRight) * kVelocityMultiplier;
            if (Math.abs(velocityInput) > kDeadBand) {
                mControlMode = ControlMode.SMART_VELOCITY;
                mReference = (velocityInput - Math.signum(velocityInput) * kDeadBand) * mConfig.master.gains.v;
            } else {
                mReference = 0.0;
            }
        }
        if (mInput.getStartButton())
            mExtendSolenoid = !mExtendSolenoid;
    }

    private void setSetPoint(double setPoint) {
        mControlMode = ControlMode.SMART_MOTION;
        mReference = setPoint;
    }

    private void configureSoftLimit(CANSparkMax spark, SoftLimitDirection direction, Float configLimit) {
        var limit = Optional.ofNullable(configLimit);
        check(spark.enableSoftLimit(direction, limit.isPresent()), "enable soft limit");
        limit.ifPresent(softLimit -> check(spark.setSoftLimit(direction, softLimit), "set soft limit"));
    }

    private Controller setupController(SimpleConfig config) {
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
        slave.follow(master);
        return slave;
    }

    private void check(CANError error, String name) {
        if (error != CANError.kOk) {
            var message = String.format("Failed to set %s! Error: %s", name, error);
            System.err.println(message);
            throw new RuntimeException(message);
        }
    }

    private void scoldUser() {
        System.err.println("Use test mode!");
    }
}
