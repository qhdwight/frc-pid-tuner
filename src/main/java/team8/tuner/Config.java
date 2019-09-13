package team8.tuner;

import java.util.ArrayList;

class Config extends AbstractConfig {
    public static class SparkConfig {
        public int id;
        public boolean isInverted;
    }

    public static class MasterSparkConfig extends SparkConfig {
        public double ramp, p, i, d, f, ff, a, v, armComOffset;
        public Double armFf;
        public double voltageCompensation = 12.0;
        public double positionConversion = 1.0, velocityConversion = 1.0;
        public Float forwardLimit, reverseLimit;
        public boolean isBraked = true;
    }

    public MasterSparkConfig master;
    public ArrayList<SparkConfig> slaves = new ArrayList<>();

    public int xboxId;
    public boolean writeCsv = true;
    public double aSetPoint, bSetPoint, xSetPoint, ySetPoint;
}
