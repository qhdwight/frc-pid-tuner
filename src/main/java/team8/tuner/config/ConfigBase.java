package team8.tuner.config;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;

public abstract class ConfigBase {

	@Override
	public String toString() {
		try {
			return new ObjectMapper().writerWithDefaultPrettyPrinter().writeValueAsString(this);
		} catch (IOException exception) {
			exception.printStackTrace();
			return super.toString();
		}
	}
}
