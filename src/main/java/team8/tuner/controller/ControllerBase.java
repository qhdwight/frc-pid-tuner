package team8.tuner.controller;

import java.util.function.Function;

public abstract class ControllerBase<TController> implements Controller {

	protected TController mController;

	ControllerBase(int deviceId) {
		mController = controllerFactory().apply(deviceId);
	}

	abstract Function<Integer, TController> controllerFactory();
}
