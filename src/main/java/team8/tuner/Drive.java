package team8.tuner;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Joystick;

import java.util.List;

import static team8.tuner.Robot.kDeadBand;

public class Drive {

	TalonFX mLM = new TalonFX(12), mLS = new TalonFX(13), mRM = new TalonFX(2), mRS = new TalonFX(3);
	Joystick mTS = new Joystick(1), mDS = new Joystick(0);
	List<TalonFX> mTalons = List.of(mLM, mLS, mRM, mRS);

	Drive() {
		for (TalonFX talon : mTalons) {
			talon.configFactoryDefault();
		}
		mLS.follow(mLM);
		mRS.follow(mRM);
	}

	void update() {
		double throttle = mDS.getY(), wheel = mTS.getX();
		if (Math.abs(throttle) < kDeadBand) {
			throttle = 0.0;
		}
		if (Math.abs(wheel) < kDeadBand) {
			wheel = 0.0;
		}
		mRM.set(TalonFXControlMode.PercentOutput, throttle + wheel);
		mLM.set(TalonFXControlMode.PercentOutput, throttle - wheel);
	}
}
