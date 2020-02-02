package team8.tuner.controller;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import team8.tuner.config.Config.SimpleConfig;

import java.util.function.Function;

public class Victor extends CTREControllerBase<VictorSPX> {

	public Victor(SimpleConfig config) {
		super(config);
	}

	@Override
	Function<Integer, VictorSPX> controllerFactory() {
		return VictorSPX::new;
	}

	@Override
	public double getOutputCurrent() {
		return 0.0;
	}
}
