package team8.tuner.controller;

public interface Controller {

    void follow(Controller master);

    void setOutput(ControlMode controlMode, double reference, double arbitraryFeedForward);

    double getOutputCurrent();

    double getPosition();

    double getVelocity();

    double getAppliedOutput();

    public enum ControlMode {
        DISABLED, SMART_MOTION, SMART_VELOCITY, PERCENT_OUTPUT
    }
}
