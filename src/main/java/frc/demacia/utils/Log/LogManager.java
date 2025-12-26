// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
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
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

/**
 * Centralized logging system for robot telemetry and diagnostics.
 * 
 * <p>Features:</p>
 * <ul>
 *   <li>Automatic data logging to file</li>
 *   <li>NetworkTables publishing for dashboard viewing</li>
 *   <li>Smart log entry grouping for performance</li>
 *   <li>Hot-reload compatible</li>
 *   <li>Competition-aware (can disable NT in comp)</li>
 * </ul>
 * 
 * <p><b>Usage:</b></p>
 * <pre>
 * // In RobotContainer constructor
 * new LogManager();
 * 
 * // In subsystems
 * LogManager.addEntry("DriveLeft/Voltage", () -> leftMotor.getCurrentVoltage())
 *     .withLogLevel(LogLevel.LOG_AND_NT)
 *     .build();
 * </pre>
 */
public class LogManager extends SubsystemBase {

  public static LogManager logManager;

  DataLog log;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;

  ArrayList<LogEntry<?>> individualLogEntries = new ArrayList<>();
  
  LogEntry<?>[] categoryLogEntries = new LogEntry<?>[16];

  private Map<String, String[]> nameSplitCache = new HashMap<>();
  private Map<String, Integer[]> entryLocationMap = new HashMap<>();

  public LogManager() {
    logManager = this;

    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }


  /**
   * Creates a log entry builder for StatusSignal-based logging (CTRE Phoenix 6).
   * 
   * <p>StatusSignals provide timestamp synchronization and efficient data transfer.</p>
   * 
   * @param name Log entry name (use "/" for hierarchy, e.g., "Drive/Left/Current")
   * @param statusSignals One or more CTRE StatusSignal objects
   * @return Builder for configuring the log entry
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, StatusSignal<T>... statusSignals) {
    // return null;
      return new LogEntryBuilder<T>(name, statusSignals);
  }

  /**
   * Creates a log entry builder for Supplier-based logging (REV, generic data).
   * 
   * <p>Suppliers are called each cycle to get fresh data.</p>
   * 
   * @param name Log entry name (use "/" for hierarchy)
   * @param suppliers One or more Supplier functions
   * @return Builder for configuring the log entry
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, Supplier<T>... suppliers) {
    return new LogEntryBuilder<T>(name, suppliers);
  }

  /**
   * Removes NetworkTables publishers for entries marked LOG_ONLY_NOT_IN_COMP.
   * 
   * <p>Call this when entering competition mode to reduce network traffic
   * and improve loop timing. Data continues to be logged to file.</p>
   */
  public static void removeInComp() {
    for (int i = 0; i < logManager.individualLogEntries.size(); i++) {
      logManager.individualLogEntries.get(i).removeInComp();
      if (logManager.individualLogEntries.get(i).logLevel == LogLevel.LOG_ONLY_NOT_IN_COMP) {
        logManager.individualLogEntries.remove(logManager.individualLogEntries.get(i));
        i--;
      }
    }

    for (int i = 0; i < 4; i++) {
      if (logManager.categoryLogEntries[i] != null && logManager.categoryLogEntries[i] != null) {
        logManager.categoryLogEntries[i].removeInComp();
        if (logManager.categoryLogEntries[i].logLevel == LogLevel.LOG_ONLY_NOT_IN_COMP) {
          logManager.categoryLogEntries[i] = null;
          int index = i;
          logManager.entryLocationMap.entrySet().removeIf(entry -> entry.getValue()[0] == index);
        }
      }
    }
  }
  
  /**
   * Clears all log entries. Useful for testing or hot-reload scenarios.
   */
  public static void clearEntries() {
    if (logManager != null) {
      logManager.individualLogEntries.clear();
      for (int i = 0; i < logManager.categoryLogEntries.length; i++) {
        if (logManager.categoryLogEntries[i] != null) {
          logManager.categoryLogEntries[i] = null;
        }
      }
      logManager.entryLocationMap.clear();
      logManager.nameSplitCache.clear();
    }
  }
  
  /**
   * Gets the total number of active log entries.
   * 
   * @return Number of entries being logged
   */
  public static int getEntryCount() {
    if (logManager == null) return 0;

    int count = logManager.individualLogEntries.size();
    for (LogEntry<?> entry2 : logManager.categoryLogEntries) {
      if (entry2 != null) {
        count++;
      }
    }
    return count;
  }

  /**
   * Finds a log entry by name.
   * 
   * @param name The entry name to search for
   * @return The log entry, or null if not found
   */
  public static LogEntry<?> findEntry(String name) {
    if (logManager == null) return null;
    
    Integer[] location = logManager.entryLocationMap.get(name);
    if (location == null) return null;
    
    int categoryIndex = location[0];
    
    if (categoryIndex == -1) {
      int index = location[0];
      if (index >= 0 && index < logManager.individualLogEntries.size()) {
        return logManager.individualLogEntries.get(index);
      }
    } else {
      return logManager.categoryLogEntries[categoryIndex];
    }
    
    return null;
  }

  /**
   * Removes a log entry by name.
   * 
   * <p>Useful for hot-reload or dynamic logging scenarios.</p>
   * 
   * @param name The entry name to remove
   * @return true if entry was found and removed, false otherwise
   */
  public static boolean removeEntry(String name) {
      if (logManager == null) return false;
      
      Integer[] location = logManager.entryLocationMap.get(name);
      if (location == null) return false;
      
      int categoryIndex = location[0];
      
      if (categoryIndex == -1) {
          int index = location[1];
          if (index >= 0 && index < logManager.individualLogEntries.size()) {
              logManager.individualLogEntries.remove(index);
              logManager.entryLocationMap.remove(name);
              logManager.nameSplitCache.remove(name);
              
              logManager.updateIndicesAfterRemoval(index);
              return true;
          }
      } else {
          int subIndex = location[1];
          int dataIndex = location[2];
          int dataCount = location[3];
          
          logManager.entryLocationMap.remove(name);
          logManager.nameSplitCache.remove(name);
          
          LogEntry<?> categoryEntry = logManager.categoryLogEntries[categoryIndex];
          if (categoryEntry != null) {
            categoryEntry.removeData(subIndex, dataIndex, dataCount);
            
            logManager.updateSubIndicesAfterRemoval(categoryIndex, subIndex);
            logManager.updateDataIndicesAfterRemoval(categoryIndex, dataIndex, dataCount);
            
            if (categoryEntry.data == null || 
              categoryEntry.name == null || 
              categoryEntry.name.trim().isEmpty()) {
              logManager.categoryLogEntries[categoryIndex] = null;
              final int catIndex = categoryIndex;
              logManager.entryLocationMap.entrySet().removeIf(
                  entry -> entry.getValue()[0] == catIndex
              );
            }
          }
          
          return true;
      }
      
      return false;
  }

  /**
   * Logs a message with specified alert level.
   * 
   * <p>Messages are written to:</p>
   * <ul>
   *   <li>Driver Station console</li>
   *   <li>Log file</li>
   *   <li>On-screen alerts (for warnings/errors)</li>
   * </ul>
   * 
   * @param message The message to log
   * @param alertType Severity level (kInfo, kWarning, kError)
   * @return ConsoleAlert object for additional control
   */
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

  /**
   * Logs an info-level message.
   * 
   * @param message The message to log
   * @return ConsoleAlert object
   */
  public static ConsoleAlert log(Object message) {
    return log(message, AlertType.kInfo);
  }

  @Override
  public void periodic() {
    for (LogEntry<?> e : individualLogEntries) {
      e.log();
    }
    for (LogEntry<?> e : categoryLogEntries) {
      if (e != null){
        e.log();
      }
    }
  }

  public <T> LogEntry<T> add(String name, Data<T> data, LogLevel logLevel, String metaData) {
    LogEntry<T> entry = null;

    int categoryIndex = getCategoryIndex(data, logLevel, metaData);

    if (categoryIndex == -1){
      entry = new LogEntry<T>(name, data, logLevel, metaData);
      individualLogEntries.add(entry);
      int index = individualLogEntries.size() - 1;
      entryLocationMap.put(name, new Integer[] {-1, index, null, null});
    } else{
      entry = addToEntryArray(categoryIndex, name, data);
    }

    return entry;
  }

  @SuppressWarnings("unchecked")
  private <T> LogEntry<T> addToEntryArray(int i, String name, Data<T> data) {
    int subIndex;
    int dataIndex = 0;
    
    if (categoryLogEntries[i] == null || categoryLogEntries[i].data == null) {
        categoryLogEntries[i] = new LogEntry<>(name, data, 
        i <= 3? LogLevel.LOG_ONLY_NOT_IN_COMP: i <= 7? LogLevel.LOG_ONLY: i <= 11? LogLevel.LOG_AND_NT_NOT_IN_COMP: LogLevel.LOG_AND_NT
        , "");
        subIndex = 1;
        dataIndex = 0;
    } else {
        String[] parts = getCachedSplit(categoryLogEntries[i].name);
        subIndex = parts.length + 1;

        if (categoryLogEntries[i].data.getSignals() != null) {
          dataIndex = categoryLogEntries[i].data.getSignals().length;
        } else if (categoryLogEntries[i].data.getSuppliers() != null) {
          dataIndex = categoryLogEntries[i].data.getSuppliers().length;
        }

        try {
            ((LogEntry<T>) categoryLogEntries[i]).addData(name, data);
        } catch (Exception e) {
            LogManager.log("Error combining log entries: " + e.getMessage(), AlertType.kError);
        }
    }

    int dataLength = 0;
    if (data.getSignals() != null) {
      dataLength = data.getSignals().length;
    } else if (data.getSuppliers() != null) {
      dataLength = data.getSuppliers().length;
    }


    entryLocationMap.put(name, new Integer[]{i, subIndex, dataIndex, dataLength});
    
    return (LogEntry<T>) categoryLogEntries[i];
  }

  // Cache split results
  private String[] getCachedSplit(String name) {
    return nameSplitCache.computeIfAbsent(name, k -> k.split(" \\| "));
  }

  private int getCategoryIndex(Data<?> data, LogLevel logLevel, String metaData) {
    boolean isSignal = data.getSignals() != null;
    boolean isSupplier = data.getSuppliers() != null;
    boolean isDouble = data.isDouble();
    boolean isBoolean = data.isBoolean();
    
    if (!(isDouble || isBoolean) || !(isSignal || isSupplier) || metaData != "") {
      return -1;
    }
    
    int baseIndex = (isSignal ? 0 : 2) + (isDouble ? 0 : 1);
    int levelOffset;
    switch (logLevel) {
      case LOG_ONLY_NOT_IN_COMP:
        levelOffset = 0;
        break;
      case LOG_ONLY:
        levelOffset = 4;
        break;
      case LOG_AND_NT_NOT_IN_COMP:
        levelOffset = 8;
        break;
      default:
        levelOffset = 12;
        break;
    }
    
    return baseIndex + levelOffset;
  }

  private void updateIndicesAfterRemoval(int removedIndex) {
      for (Map.Entry<String, Integer[]> entry : entryLocationMap.entrySet()) {
          Integer[] location = entry.getValue();
          if (location[0] == -1 && location[1] > removedIndex) {
              entry.setValue(new Integer[]{-1, location[1] - 1, location[2], location[3]});
          }
      }
  }

  private void updateSubIndicesAfterRemoval(int categoryIndex, int removedSubIndex) {
      for (Map.Entry<String, Integer[]> entry : entryLocationMap.entrySet()) {
        Integer[] location = entry.getValue();
          if (location[0] == categoryIndex && location[2] > removedSubIndex) {
              entry.setValue(new Integer[]{categoryIndex, location[1] - 1, location[2], location[3]});
          }
      }
  }

  private void updateDataIndicesAfterRemoval(int categoryIndex, int removedDataIndex, int removedCount) {
      for (Map.Entry<String, Integer[]> entry : entryLocationMap.entrySet()) {
          Integer[] location = entry.getValue();
          if (location[0] == categoryIndex && 
              location[2] != null &&
              location[2] >= removedDataIndex + removedCount) {
              entry.setValue(new Integer[] {location[0], location[1], location[2] - removedCount, location[3]});
          }
      }
  }


  @SuppressWarnings("unused")
  private LogEntry<?> get(String name) {
    LogEntry<?> e = find(name);
    return e != null 
    ?e 
    :new LogEntry<>(name, null, LogLevel.LOG_ONLY_NOT_IN_COMP, "");
  }

  private LogEntry<?> find(String name) {
    for (LogEntry<?> entry : individualLogEntries) {
      if (entry.name.equals(name)) {
        return entry;
      }
    }
    return null;
  }
}