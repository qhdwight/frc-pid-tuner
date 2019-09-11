package team8.tuner;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import team8.tuner.Config.MasterSparkConfig;
import team8.tuner.Config.SparkConfig;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

    private static final int PID_SLOT_ID = 0;

    private Config m_Config;
    private CANSparkMax m_Master;
    private CANEncoder m_MasterEncoder;
    private List<CANSparkMax> m_Slaves;
    private XboxController m_Controller;
    private PowerDistributionPanel pdp;

    @Override
    public void robotInit() {
        System.out.printf("Initializing PID tuner with:%n%s%n", C.getJson(Config.class));
        pdp = new PowerDistributionPanel(0);
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
        m_Config = C.readGenericConfig(Config.class);
        m_Controller = new XboxController(m_Config.xboxId);
        System.out.printf("Using X-Box controller with id: %d%n", m_Config.xboxId);
        // Master
        ifValid(m_Master, CANSparkMax::close);
        m_Master = setupMaster(m_Config.master);
        m_MasterEncoder = m_Master.getEncoder();
        // Slaves
        ifValid(m_Slaves, slaves -> slaves.forEach(CANSparkMax::close));
        m_Slaves = m_Config.slaves.stream()
                .map(slaveConfig -> setupSlave(slaveConfig, m_Master))
                .collect(Collectors.toList());
    }

    @Override
    public void testPeriodic() {
        if (m_Controller.getAButtonPressed()) {
            setSetPoint(m_Config.aSetPoint);
        } else if (m_Controller.getBButtonPressed()) {
            setSetPoint(m_Config.bSetPoint);
        } else if (m_Controller.getXButtonPressed()) {
            setSetPoint(m_Config.xSetPoint);
        } else if (m_Controller.getYButtonPressed()) {
            setSetPoint(m_Config.ySetPoint);
        } else if (m_Controller.getBackButtonPressed()) {
            m_Master.disable();
            System.out.println("Disabling...");
        }
        if (m_Config.writeCsv) {
            pdp.getTotalCurrent();
            CSVWriter.addData("totalCurrent", pdp.getTotalCurrent());
            CSVWriter.addData("current", m_Master.getOutputCurrent() + m_Slaves.get(0).getOutputCurrent());
            CSVWriter.addData("output", m_Master.getAppliedOutput());
            CSVWriter.addData("position", m_MasterEncoder.getPosition());
            CSVWriter.addData("velocity", m_MasterEncoder.getVelocity());
        }
    }

    @Override
    public void disabledInit() {
        ifValid(m_Master, CANSparkMax::disable);
        ifValid(m_Slaves, slaves -> slaves.forEach(CANSparkMax::disable));
        ifValid(m_Config, config -> {
            if (m_Config.writeCsv) CSVWriter.write();
        });
    }

    private void setSetPoint(double setPoint) {
        System.out.printf("Setting set point to %s with ff %s%n", setPoint, m_Config.master.ff);
        check(m_Master.getPIDController().setReference(setPoint, ControlType.kSmartMotion, PID_SLOT_ID, m_Config.master.ff, ArbFFUnits.kPercentOut));
//        m_Master.getPIDController().setReference(setPoint, ControlType.kSmartMotion, PID_SLOT_ID, m_Config.master.ff * 12);
    }

    private CANSparkMax setupSpark(int id) {
        final var spark = new CANSparkMax(id, MotorType.kBrushless);
        System.out.printf("Setup spark with id %d%n", id);
        check(spark.restoreFactoryDefaults());
        check(spark.getEncoder().setPosition(0.0));
        return spark;
    }

    private void configureLimit(CANSparkMax spark, SoftLimitDirection direction, Float configLimit) {
        final var limit = Optional.ofNullable(configLimit);
        check(spark.setSoftLimit(direction, limit.orElse(0.0f)));
        check(spark.enableSoftLimit(direction, limit.isPresent()));
    }

    private CANSparkMax setupMaster(final MasterSparkConfig config) {
        final var spark = setupSpark(config.id);
        configureLimit(spark, SoftLimitDirection.kForward, config.forwardLimit);
        configureLimit(spark, SoftLimitDirection.kReverse, config.reverseLimit);
        spark.setInverted(config.isInverted);
        check(spark.setIdleMode(config.isBraked ? IdleMode.kBrake : IdleMode.kCoast));
        check(spark.enableVoltageCompensation(config.voltageCompensation));
        check(spark.setClosedLoopRampRate(config.ramp));
        final var controller = spark.getPIDController();
        System.out.printf("Using gains %nP: %s %nI: %s %nD: %s %nF: %s%n%nV: %s %nA: %s%n", config.p, config.i, config.d, config.f, config.v, config.a);
        check(controller.setP(config.p, PID_SLOT_ID));
        check(controller.setI(config.i, PID_SLOT_ID));
        check(controller.setD(config.d, PID_SLOT_ID));
        check(controller.setFF(config.f, PID_SLOT_ID));
        check(controller.setIMaxAccum(0.0, PID_SLOT_ID));
        check(controller.setOutputRange(-1.0, 1.0, PID_SLOT_ID));
        check(controller.setSmartMotionMaxAccel(config.a, PID_SLOT_ID));
        check(controller.setSmartMotionMaxVelocity(config.v, PID_SLOT_ID));
        check(controller.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, PID_SLOT_ID));
        check(controller.setSmartMotionAllowedClosedLoopError(0.0, PID_SLOT_ID));
        check(controller.setSmartMotionMinOutputVelocity(0.0, PID_SLOT_ID));
        final var encoder = spark.getEncoder();
        check(encoder.setPositionConversionFactor(config.positionConversion));
        check(encoder.setVelocityConversionFactor(config.velocityConversion));
        return spark;
    }

    private CANSparkMax setupSlave(final SparkConfig config, final CANSparkMax master) {
        final var spark = setupSpark(config.id);
        check(spark.setIdleMode(master.getIdleMode()));
        check(spark.follow(master, config.isInverted));
        return spark;
    }

    private <T> void ifValid(final T object, final Consumer<T> action) {
        Optional.ofNullable(object).ifPresent(action);
    }

    private void check(final CANError error) {
        if (error != CANError.kOk) {
            final var message = String.format("Failed to set! Error: %s", error);
            System.err.println(message);
            throw new RuntimeException(message);
        }
    }

    private void scoldUser() {
        System.err.println("Use test mode!");
    }
}
