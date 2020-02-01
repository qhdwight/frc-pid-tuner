package team8.tuner.controller;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import team8.tuner.config.Config.SimpleConfig;

public abstract class TalonControllerBase<TController extends BaseTalon> extends CTREControllerBase<TController> {

    TalonControllerBase(SimpleConfig config) {
        super(config);
    }

    @Override
    public double getOutputCurrent() {
        return mController.getSupplyCurrent();
    }
}
