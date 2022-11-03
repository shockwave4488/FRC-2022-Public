package frc.lib.logging;

import frc.lib.wpiextensions.TimerHelper;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.Charset;

public class StringTracker {

  private final StringTrackable stringTarget;
  private final int frequency;
  private final String name;
  private final String header;

  private int lastWritten;
  private BufferedWriter file;
  private static final String valueSeparator = ",";

  public StringTracker(StringTrackable stringTarget, String name, int frequency, String header) {
    this.stringTarget = stringTarget;
    this.frequency = frequency;
    this.header = header;
    this.name = name;
  }

  public void createFile(String dir) {
    try {
      String path = dir + "/" + name + ".txt";
      file = new BufferedWriter(new FileWriter(path, Charset.forName("UTF-8")));
      file.write(header + "\n");
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void closeWriter() {
    try {
      file.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void update(String timestamp) {
    if (file == null) {
      System.out.println("Writing to log before initializing it!");
      return;
    }

    if (getTimeMillis() - lastWritten > (1000 / frequency)) {
      lastWritten = getTimeMillis();
      try {
        file.write(timestamp + valueSeparator + stringTarget.get() + "\n");
      } catch (IOException e) {
        e.printStackTrace();
      }
    }
  }

  public void flush() {
    try {
      file.flush();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  private int getTimeMillis() {
    return TimerHelper.getMatchTimeMillis();
  }
}
