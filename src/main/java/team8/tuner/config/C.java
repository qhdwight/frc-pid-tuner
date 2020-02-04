package team8.tuner.config;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

/**
 * Config storage using JSON
 *
 * @author Quintin Dwight
 */
public class C {

	public static final String kConfigFolderName = "config";
	private static final Path kConfigPath = RobotBase.isReal()
			? Paths.get(Filesystem.getDeployDirectory().toString(), kConfigFolderName)
			: Paths.get(Filesystem.getLaunchDirectory().toString(), "src", "main", "deploy", kConfigFolderName);
	private static ObjectMapper sMapper = new ObjectMapper();

	private C() {
	}

	public static <T extends ConfigBase> T read(Class<T> configClass, String fileName) {
		File configFile = getFileForConfig(fileName);
		String configClassName = configClass.getSimpleName();
		if (!configFile.exists()) {
			System.err.printf("A default config file was not found for %s", configClassName);
			throw new IllegalArgumentException();
		}
		try {
			return sMapper.readValue(configFile, configClass);
		} catch (IOException readException) {
			System.err.printf("An error occurred trying to read config for class %s%n", configClassName);
			readException.printStackTrace();
			throw new IllegalArgumentException();
		}
	}

	private static File getFileForConfig(String fileName) {
		return Paths.get(kConfigPath.toString(), String.format("%s.json", fileName)).toFile();
	}
}