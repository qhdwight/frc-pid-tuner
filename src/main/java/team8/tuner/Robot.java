package team8.tuner;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
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
    private List<CANSparkMax> m_Slaves;
    private XboxController m_Controller;

    @Override
    public void robotInit() {
        System.out.printf("Initializing PID tuner with:%n%s%n", C.getJson(Config.class));
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
        m_Config = C.get(Config.class);
        m_Controller = new XboxController(m_Config.xboxId);
        // Master
        ifValid(m_Master, CANSparkMax::close);
        m_Master = setupMaster(m_Config.master);
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
        }
    }

    @Override
    public void disabledInit() {
        ifValid(m_Master, CANSparkMax::disable);
        ifValid(m_Slaves, slaves -> slaves.forEach(CANSparkMax::disable));
    }

    private void setSetPoint(double setPoint) {
        m_Master.getPIDController().setReference(setPoint, ControlType.kSmartMotion, PID_SLOT_ID, m_Config.master.ff, ArbFFUnits.kPercentOut);
    }

    private CANSparkMax setupSpark(int id) {
        final var spark = new CANSparkMax(id, MotorType.kBrushless);
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
            final var message = "Failed to set!";
            System.err.println(message);
            throw new RuntimeException(message);
        }
    }

    private void scoldUser() {
        System.err.println("Use test mode!");
    }
}
