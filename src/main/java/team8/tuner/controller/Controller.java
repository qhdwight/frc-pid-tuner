package team8.tuner.controller;

public interface Controller {

	enum ControlMode {
		DISABLED, SMART_MOTION, SMART_VELOCITY, PERCENT_OUTPUT
	}

	void follow(Controller master);

	void setOutput(ControlMode controlMode, double reference, double arbitraryFeedForward);

	double getOutputCurrent();

	double getPosition();

	double getVelocity();

	double getAppliedOutput();
}
