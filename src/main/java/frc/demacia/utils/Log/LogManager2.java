// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Data;
import frc.demacia.utils.constants.UtilsContants.ConsoleConstants;

public class LogManager2 extends SubsystemBase {

  public static LogManager2 logManager;

  DataLog log;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;
  
  private static int SkipedCycles = 0;
  private static int SKIP_CYCLES = 1;
  private static boolean isLoggingEnabled = true;

  ArrayList<LogEntry2<?>> logEntries = new ArrayList<>();

  public LogManager2() {
    logManager = this;

    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }

  public static <T> LogEntryBuilder<T> addEntry(String name, StatusSignal<T> ... statusSignals) {
      return new LogEntryBuilder<T>(name, statusSignals);
  }

  public static <T> LogEntryBuilder<T> addEntry(String name, Supplier<T> ... suppliers) {
    return new LogEntryBuilder<T>(name, suppliers);
  }

  public static void removeInComp() {
    for (int i = 0; i < logManager.logEntries.size(); i++) {
      logManager.logEntries.get(i).removeInComp();
      if (logManager.logEntries.get(i).logLevel == 1) {
        logManager.logEntries.remove(logManager.logEntries.get(i));
        i--;
      }
    }
  }
  
  public static void clearEntries() {
    if (logManager != null) {
      logManager.logEntries.clear();
    }
  }
  
  public static int getEntryCount() {
    return logManager != null ? logManager.logEntries.size() : 0;
  }

  public static ConsoleAlert log(Object message, AlertType alertType) {
    DataLogManager.log(String.valueOf(message));
    
    ConsoleAlert alert = new ConsoleAlert(String.valueOf(message), alertType);
    alert.set(true);
    if (activeConsole.size() > ConsoleConstants.CONSOLE_LIMIT) {
      activeConsole.get(0).close();
      activeConsole.remove(0);
    }
    activeConsole.add(alert);
    return alert;
  }

  public static ConsoleAlert log(Object meesage) {
    return log(meesage, AlertType.kInfo);
  }

  public static void setStaticSkipCycles(int cycles) {
    SKIP_CYCLES = Math.max(1, cycles);
  }

  public static int getStaticSkipCycles() {
    return SKIP_CYCLES;
  }
  
  public static void setLoggingEnabled(boolean isenabled) {
    isLoggingEnabled = isenabled;
  }

  public static boolean getLoggingEnabled() {
    return isLoggingEnabled;
  }

  @Override
  public void periodic() {
    SkipedCycles++;
    if (SkipedCycles < SKIP_CYCLES || !isLoggingEnabled) {
      return;
    }
    SkipedCycles = 0;

    for (LogEntry2<?> e : logEntries) {
      e.log();
    }
  }

  public <T> LogEntry2<T> add(String name, Data<T> data, int logLevel, String metaData) {
    LogEntry2<T> entry = new LogEntry2<T>(name, data, logLevel, metaData);
    logEntries.add(entry);
    return entry;
  }

  private LogEntry2<?> get(String name) {
    LogEntry2<?> e = find(name);
    return e != null 
    ?e 
    :new LogEntry2<>(name, null, 1, "");
  }

  private LogEntry2<?> find(String name) {
    for (LogEntry2<?> entry : logEntries) {
      if (entry.name.equals(name)) {
        return entry;
      }
    }
    return null;
  }
}