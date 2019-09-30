package team8.tuner;

import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.*;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import team8.tuner.configv2.C;
import team8.tuner.configv2.Config;
import team8.tuner.configv2.Config.MasterSparkConfig;
import team8.tuner.configv2.Config.SparkConfig;
import team8.tuner.csv.CSVWriter;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

    enum ControlMode {
        DISABLED, SMART_MOTION, SMART_VELOCITY, PERCENT_OUTPUT
    }

    private static final int PID_SLOT_ID = 0;
    private static final double JOYSTICK_THRESHOLD = 0.07;

    private Config m_Config;
    private CANSparkMax m_Master;
    private CANPIDController m_MasterController;
    private CANEncoder m_MasterEncoder;
    private List<CANSparkMax> m_Slaves;
    private XboxController m_Input;
    private PowerDistributionPanel m_PowerDistributionPanel;
    private double m_SetPoint, m_Velocity, m_PercentOutput;
    private boolean m_RunningConstantPercentOutput;
    private ControlMode m_ControlMode = ControlMode.DISABLED;

    @Override
    public void robotInit() {
        m_PowerDistributionPanel = new PowerDistributionPanel(0);
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
        m_Config = C.read(Config.class);
        applyConfig();
    }

    private void applyConfig() {
        System.out.printf("Initializing PID tuner with:%n%s%n", m_Config);
        m_Input = new XboxController(m_Config.xboxId);
        System.out.printf("Using X-Box controller with id: %d%n", m_Config.xboxId);
        /* Master */
        ifValid(m_Master, CANSparkMax::close); // TODO closing does not even do anything?
        m_Master = setupMaster(m_Config.master);
        check(m_Master.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed).enableLimitSwitch(false), "reverse limit");
        check(m_Master.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false), "forward limit");
        m_MasterEncoder = m_Master.getEncoder();
        m_MasterController = m_Master.getPIDController();
        /* Slaves */
        ifValid(m_Slaves, slaves -> slaves.forEach(CANSparkMax::close));
        m_Slaves = m_Config.slaves.stream()
                .map(slaveConfig -> setupSlave(slaveConfig, m_Master))
                .collect(Collectors.toList());
    }

    @Override
    public void testPeriodic() {
        handleInput();
        /* CSV Data */
        if (m_Config.writeCsv) {
            CSVWriter.addData("totalCurrent", m_PowerDistributionPanel.getTotalCurrent());
            CSVWriter.addData("sparkCurrent", m_Master.getOutputCurrent() + m_Slaves.stream().mapToDouble(CANSparkMax::getOutputCurrent).sum());
            CSVWriter.addData("output", m_Master.getAppliedOutput());
            CSVWriter.addData("position", m_MasterEncoder.getPosition());
            CSVWriter.addData("velocity", m_MasterEncoder.getVelocity());
        }
        /* Sending output to controllers */
        switch (m_ControlMode) {
            case SMART_MOTION:
            case SMART_VELOCITY:
                double arbitraryFeedForward = m_Config.master.gains.ff;
                final double angle = m_MasterEncoder.getPosition();
                if (m_Config.master.armFf != null) {
                    arbitraryFeedForward += m_Config.master.armFf * Math.cos(Math.toRadians(angle - m_Config.master.armComOffset));
                }
                double reference;
                ControlType controlType;
                switch (m_ControlMode) {
                    case SMART_MOTION:
                        reference = m_SetPoint;
                        controlType = ControlType.kSmartMotion;
                        break;
                    case SMART_VELOCITY:
                        reference = m_Velocity;
                        controlType = ControlType.kSmartVelocity;
                        break;
                    default:
                        throw new RuntimeException();
                }
                m_MasterController.setReference(
                        reference,
                        controlType, PID_SLOT_ID,
                        arbitraryFeedForward, ArbFFUnits.kPercentOut
                );
                break;
            case PERCENT_OUTPUT:
                m_Master.set(m_PercentOutput);
                break;
            case DISABLED:
                m_Master.disable();
                break;
        }
    }

    @Override
    public void disabledInit() {
        m_ControlMode = ControlMode.DISABLED;
        ifValid(m_Master, CANSparkMax::disable);
        ifValid(m_Slaves, slaves -> slaves.forEach(CANSparkMax::disable));
        ifValid(m_Config, config -> {
            if (m_Config.writeCsv) CSVWriter.write();
        });
    }

    private void handleInput() {
        if (m_Input.getAButtonPressed()) {
            setSetPoint(m_Config.aSetPoint);
        } else if (m_Input.getBButtonPressed()) {
            setSetPoint(m_Config.bSetPoint);
        } else if (m_Input.getXButtonPressed()) {
            setSetPoint(m_Config.xSetPoint);
        } else if (m_Input.getYButtonPressed()) {
            setSetPoint(m_Config.ySetPoint);
        } else if (m_Input.getBumperPressed(Hand.kRight)) {
            m_ControlMode = ControlMode.PERCENT_OUTPUT;
            m_PercentOutput = 0.2 + m_Config.master.gains.ff;
            m_RunningConstantPercentOutput = true;
        } else if (m_Input.getBumperPressed(Hand.kLeft)) {
            m_ControlMode = ControlMode.DISABLED;
            System.out.println("Disabling...");
        } else {
            double percentOutInput = m_Input.getY(Hand.kLeft) * -0.4;
            if (Math.abs(percentOutInput) > JOYSTICK_THRESHOLD) {
                m_PercentOutput = percentOutInput - Math.signum(percentOutInput) * JOYSTICK_THRESHOLD;
                m_ControlMode = ControlMode.PERCENT_OUTPUT;
                m_RunningConstantPercentOutput = false;
            } else {
                if (!m_RunningConstantPercentOutput) m_PercentOutput = 0.0;
            }
            double velocityInput = m_Input.getY(Hand.kRight) * -0.7;
            if (Math.abs(velocityInput) > JOYSTICK_THRESHOLD) {
                m_Velocity = (velocityInput - Math.signum(velocityInput) * JOYSTICK_THRESHOLD) * m_Config.master.gains.v;
                m_ControlMode = ControlMode.SMART_VELOCITY;
            } else {
                m_Velocity = 0.0;
            }
        }
    }

    private void setSetPoint(double setPoint) {
        m_SetPoint = setPoint;
        m_ControlMode = ControlMode.SMART_MOTION;
    }

    private CANSparkMax setupSpark(int id) {
        final var spark = new CANSparkMax(id, MotorType.kBrushless);
        System.out.printf("Setup spark with id %d%n", id);
        check(spark.restoreFactoryDefaults(), "factory defaults");
        return spark;
    }

    private void configureSoftLimit(final CANSparkMax spark, final SoftLimitDirection direction, final Float configLimit) {
        final var limit = Optional.ofNullable(configLimit);
        check(spark.enableSoftLimit(direction, limit.isPresent()), "enable soft limit");
        limit.ifPresent(softLimit -> check(spark.setSoftLimit(direction, softLimit), "set soft limit"));
    }

    private CANSparkMax setupMaster(final MasterSparkConfig config) {
        final var spark = setupSpark(config.id);
        configureSoftLimit(spark, SoftLimitDirection.kForward, config.forwardLimit);
        configureSoftLimit(spark, SoftLimitDirection.kReverse, config.reverseLimit);
        spark.setInverted(config.isInverted);
        check(spark.setIdleMode(config.isBraked ? IdleMode.kBrake : IdleMode.kCoast), "idle mode");
        check(spark.enableVoltageCompensation(config.voltageCompensation), "voltage compensation");
        check(spark.setClosedLoopRampRate(config.ramp), "closed loop ramp");
        final var controller = spark.getPIDController();
        check(controller.setP(config.gains.p, PID_SLOT_ID), "p");
        check(controller.setI(config.gains.i, PID_SLOT_ID), "i");
        check(controller.setD(config.gains.d, PID_SLOT_ID), "d");
        check(controller.setFF(config.gains.f, PID_SLOT_ID), "ff");
        check(controller.setIMaxAccum(0.0, PID_SLOT_ID), "max i");
        check(controller.setOutputRange(m_Config.master.minimumOutput, m_Config.master.maximumOutput, PID_SLOT_ID), "output range");
        check(controller.setSmartMotionMaxVelocity(config.gains.v, PID_SLOT_ID), "max velocity");
        check(controller.setSmartMotionMaxAccel(config.gains.a, PID_SLOT_ID), "max acceleration");
        check(controller.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, PID_SLOT_ID), "strategy");
        check(controller.setSmartMotionAllowedClosedLoopError(m_Config.master.gains.allowableError, PID_SLOT_ID), "error");
        check(controller.setSmartMotionMinOutputVelocity(0.0, PID_SLOT_ID), "min velocity");
        final var encoder = spark.getEncoder();
        check(encoder.setPositionConversionFactor(m_Config.master.positionConversion), "position conversion");
        check(encoder.setVelocityConversionFactor(m_Config.master.velocityConversion), "velocity conversion");
        check(encoder.setPosition(m_Config.master.startingPosition), "starting position");
        return spark;
    }

    private CANSparkMax setupSlave(final SparkConfig config, final CANSparkMax master) {
        final var spark = setupSpark(config.id);
        check(spark.setIdleMode(master.getIdleMode()), "idle mode");
        check(spark.follow(master, config.isInverted), "follow inverted");
        return spark;
    }

    private <T> void ifValid(final T object, final Consumer<T> action) {
        Optional.ofNullable(object).ifPresent(action);
    }

    private void check(final CANError error, final String name) {
        if (error != CANError.kOk) {
            final var message = String.format("Failed to set %s! Error: %s", name, error);
            System.err.println(message);
            throw new RuntimeException(message);
        }
    }

    private void scoldUser() {
        System.err.println("Use test mode!");
    }
}
