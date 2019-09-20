package team8.tuner.configv2;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import org.codehaus.jackson.map.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * Config storage using JSON
 *
 * @author Quintin Dwight
 */
public class C {

    private static ObjectMapper sMapper = new ObjectMapper();

    private static final Path CONFIG_FOLDER = RobotBase.isReal()
            ? Paths.get(Filesystem.getDeployDirectory().toString(), "config_v2")
            : Paths.get(Filesystem.getLaunchDirectory().toString(), "src", "main", "deploy", "config_v2");

    public static <T extends AbstractConfig> T read(Class<T> configClass) {
        File configFile = getFileForConfig(configClass);
        String configClassName = configClass.getSimpleName();
        if (!configFile.exists()) {
            System.err.printf("A default config file was not found for %s. Writing defaults...%n", configClassName);
            return saveDefaultConfig(configClass);
        }
        try {
            return sMapper.readValue(configFile, configClass);
        } catch (IOException readException) {
            System.err.printf("An error occurred trying to read config for class %s%n", configClassName);
            readException.printStackTrace();
            return saveDefaultConfig(configClass);
        }
    }

    private static File getFileForConfig(Class<? extends AbstractConfig> configClass) {
        return Paths.get(CONFIG_FOLDER.toString(), String.format("%s.json", configClass.getSimpleName())).toFile();
    }

    private static <T extends AbstractConfig> T saveDefaultConfig(Class<T> configClass) {
        try {
            T newConfig = configClass.getDeclaredConstructor().newInstance();
            try {
                writeConfig(newConfig);
                System.out.printf("Wrote defaults for %s%n", configClass.getSimpleName());
            } catch (IOException writeDefaultsException) {
                System.err.println("Error writing defaults - this should not happen!");
                writeDefaultsException.printStackTrace();
            }
            return newConfig;
        } catch (Exception createBlankException) {
            System.err.printf("Fatal error, could not create blank config for class %s. Is this a legit non-abstract class?%n", configClass.getSimpleName());
            createBlankException.printStackTrace();
            throw new RuntimeException();
        }
    }

    private static <T extends AbstractConfig> void writeConfig(T newConfig) throws IOException {
        File file = getFileForConfig(newConfig.getClass()), parentFile = file.getParentFile();
        if (!parentFile.exists()) {
            if (!parentFile.mkdirs()) throw new IOException();
        }
        sMapper.defaultPrettyPrintingWriter().writeValue(file, newConfig);
    }
}
