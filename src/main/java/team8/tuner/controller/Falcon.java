package team8.tuner.controller;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import team8.tuner.config.Config.SimpleConfig;

import java.util.function.Function;

public class Falcon extends TalonControllerBase<TalonFX> {

    public Falcon(SimpleConfig config) {
        super(config);
        mController.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, kTimeout);
        mController.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero, kTimeout);
    }

    @Override
    Function<Integer, TalonFX> controllerFactory() {
        return TalonFX::new;
    }
}
