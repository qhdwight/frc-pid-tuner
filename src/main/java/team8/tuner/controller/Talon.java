package team8.tuner.controller;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import team8.tuner.config.Config.SimpleConfig;

import java.util.function.Function;

public class Talon extends TalonControllerBase<TalonSRX> {

    public Talon(SimpleConfig config) {
        super(config);
    }

    @Override
    Function<Integer, TalonSRX> controllerFactory() {
        return TalonSRX::new;
    }
}
