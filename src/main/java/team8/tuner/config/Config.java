package team8.tuner.config;

import java.util.List;

@SuppressWarnings("squid:ClassVariableVisibilityCheck")
public class Config extends ConfigBase {

    public MasterConfig master;
    public List<SimpleConfig> slaves = List.of();
    public List<Integer> solenoidId = List.of();
    public int xboxId;
    public boolean writeCsv = true;
    public double aSetPoint, bSetPoint, xSetPoint, ySetPoint;

    public enum ControllerType {
        SPARK, FALCON, TALON, VICTOR
    }

    public static class SmartGains {
        public double p, i, iZone, iMax, d, f, ff, a, v, allowableError;
    }

    public static class SimpleConfig {
        public ControllerType type;
        public int id;
        public boolean isInverted;
    }

    public static class MasterConfig extends SimpleConfig {
        public SmartGains gains = new SmartGains();
        public Double armFf;
        public double ramp;
        public double armComOffset;
        public double voltageCompensation = 12.0;
        public double positionConversion = 1.0, velocityConversion = 1.0;
        public double minimumOutput = -1.0, maximumOutput = 1.0;
        public double startingPosition;
        public Float forwardLimit, reverseLimit;
        public boolean isBraked = true;
    }
}
