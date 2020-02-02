package team8.tuner.csv;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.UnaryOperator;

public class CSVWriter {

	private static final String kCommaDeliminator = ",", kNewLineSeparator = "\n";
	private static final int kAllocateSize = 20000;
	private static final String kFileName = "canlog.csv";

	private static final Path sCsvFile = RobotBase.isReal()
			? Paths.get("/home/lvuser", kFileName)
			: Paths.get(Filesystem.getOperatingDirectory().toString(), kFileName);
	private static final StringBuilder sBuilder = new StringBuilder(kAllocateSize);
	private static Timer sTimer = new Timer();

	private CSVWriter() {
	}

	public static void init() {
		try {
			Files.deleteIfExists(sCsvFile);
		} catch (IOException deleteException) {
			deleteException.printStackTrace();
			System.err.println("Failed to delete existing CSV file!");
		}
		sTimer.start();
	}

	private static void addData(String key, Object secondValue, UnaryOperator<StringBuilder> valueCellWriter) {
		sBuilder.append(key).append(kCommaDeliminator).append(secondValue).append(kCommaDeliminator);
		valueCellWriter.apply(sBuilder).append(kNewLineSeparator);
		if (sBuilder.length() > kAllocateSize) write();
	}

	public static void addData(String key, Object customSecond, Object value) {
		addData(key, customSecond, builder -> builder.append(value));
	}

	public static void addData(String key, Object value) {
		addData(key, sTimer.get(), builder -> builder.append(value));
	}

	static void addData(String key, double value) {
		addData(key, sTimer.get(), builder -> builder.append(value));
	}

	public static void write() {
		System.out.println("Writing CSV...");
		try (FileWriter fileWriter = new FileWriter(sCsvFile.toFile(), true)) {
			fileWriter.append(sBuilder.toString());
		} catch (IOException writeException) {
			System.err.println("Failed to write CSV:");
			writeException.printStackTrace();
		} finally {
			sBuilder.setLength(0);
		}
	}
}