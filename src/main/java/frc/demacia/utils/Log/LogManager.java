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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Data;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

/**
 * Centralized logging system for robot telemetry and diagnostics.
 */
public class LogManager extends SubsystemBase {

  private static LogManager logManager;

  public static DataLog log;
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("Log");

  private static ArrayList<ConsoleAlert> activeConsole;

  private ArrayList<LogEntry<?>> individualLogEntries = new ArrayList<>();
  
  private LogEntry<?>[] categoryLogEntries = new LogEntry<?>[16];

  private Map<String, String[]> nameSplitCache = new HashMap<>();
  private Map<String, Integer[]> entryLocationMap = new HashMap<>();
  private Map<String, Supplier<?>> supplierCache = new HashMap<>();

  private LogManager() {
    if (logManager != null) {
      CommandScheduler.getInstance().unregisterSubsystem(this);
      return;
    }
    logManager = this;
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    
    activeConsole = new ArrayList<>();
    log("log manager is ready");
  }

  static{
    if (logManager == null) {
      new LogManager();
    }
  }

  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, StatusSignal<T>... statusSignals) {
    return new LogEntryBuilder<T>(name, statusSignals);
  }

  @SuppressWarnings("unchecked")
  public static <T> LogEntryBuilder<T> addEntry(String name, Supplier<T>... suppliers) {
    return new LogEntryBuilder<T>(name, suppliers);
  }

  public static void removeInComp() {
    if (logManager == null) return; 

    for (int i = 0; i < logManager.individualLogEntries.size(); i++) {
      logManager.individualLogEntries.get(i).removeInComp();
      if (logManager.individualLogEntries.get(i).getLogLevel() == LogLevel.LOG_ONLY_NOT_IN_COMP) {
        logManager.individualLogEntries.remove(i);
        i--;
      }
    }

    for (int i = 0; i < 4; i++) {
      if (logManager.categoryLogEntries[i] != null) {
        logManager.categoryLogEntries[i].removeInComp();
        if (logManager.categoryLogEntries[i].getLogLevel() == LogLevel.LOG_ONLY_NOT_IN_COMP) {
          logManager.categoryLogEntries[i] = null;
          int index = i;
          logManager.entryLocationMap.entrySet().removeIf(entry -> entry.getValue()[0] == index);
        }
      }
    }
  }
  
  public static void clearEntries() {
    if (logManager != null) {
      logManager.individualLogEntries.clear();
      for (int i = 0; i < logManager.categoryLogEntries.length; i++) {
        logManager.categoryLogEntries[i] = null;
      }
      logManager.entryLocationMap.clear();
      logManager.nameSplitCache.clear();
      logManager.supplierCache.clear();
    }
  }
  
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

  public static LogEntry<?> findEntry(String name) {
    if (logManager == null) return null;
    
    Integer[] location = logManager.entryLocationMap.get(name);
    if (location == null) return null;
    
    int categoryIndex = location[0];
    
    if (categoryIndex == -1) {
      int index = location[1];
      if (index >= 0 && index < logManager.individualLogEntries.size()) {
        return logManager.individualLogEntries.get(index);
      }
    } else {
      return logManager.categoryLogEntries[categoryIndex];
    }
    
    return null;
  }

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
              logManager.supplierCache.remove(name);
              
              logManager.updateIndicesAfterRemoval(index);
              return true;
          }
      } else {
          int subIndex = location[1];
          int dataIndex = location[2];
          int dataCount = location[3];
          
          logManager.entryLocationMap.remove(name);
          logManager.nameSplitCache.remove(name);
          logManager.supplierCache.remove(name);
          
          LogEntry<?> categoryEntry = logManager.categoryLogEntries[categoryIndex];
          if (categoryEntry != null) {
            categoryEntry.removeData(subIndex, dataIndex, dataCount);
            
            logManager.updateSubIndicesAfterRemoval(categoryIndex, subIndex);
            logManager.updateDataIndicesAfterRemoval(categoryIndex, dataIndex, dataCount);
            
            if (categoryEntry.getData() == null || 
              categoryEntry.getName() == null || 
              categoryEntry.getName().trim().isEmpty()) {
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

  public static ConsoleAlert log(Object message) {
    return log(message, AlertType.kInfo);
  }

  @Override
  public void periodic() {
    Data.refreshAll();

    for (int i = activeConsole.size() - 1; i >= 0; i--) {
      ConsoleAlert alert = activeConsole.get(i);
      if (alert.isTimerOver()) {
          alert.set(false);
          activeConsole.remove(i);
      }
    }

    for (int i = 0; i < individualLogEntries.size(); i++) {
        individualLogEntries.get(i).log();
    }
    
    for (LogEntry<?> e : categoryLogEntries) {
      if (e != null){
        e.log();
      }
    }
  }

  public static <T> LogEntry<T> add(String name, Data<T> data, LogLevel logLevel, String metaData, boolean isSeparated) {
    LogEntry<T> entry = null;

    int categoryIndex = logManager.getCategoryIndex(data, logLevel, isSeparated);

    if (categoryIndex == -1){
      entry = new LogEntry<T>(name, data, logLevel, metaData);
      logManager.individualLogEntries.add(entry);
      int index = logManager.individualLogEntries.size() - 1;
      logManager.entryLocationMap.put(name, new Integer[] {-1, index, null, null});
    } else{
      entry = logManager.addToEntryArray(categoryIndex, name, data, metaData);
    }
    String[] nameParts = name.split(": ");
    String GroupName = nameParts[0];
    String[] parts = nameParts.length >=2 ? nameParts[1].split(", ") 
    : new String[0];
    for (int i = 0; i < Math.min(parts.length, data.getSuppliers().length); i++) {
      LogManager.log(GroupName + ": " + parts[i]);
      LogManager.log(data.getSuppliers()[i].get());
      logManager.supplierCache.put((GroupName + ": " + parts[i]).toLowerCase(), data.getSuppliers()[i]);
    }

    return entry;
  }

  public static Supplier<?> getSupplier(String name) {
    return logManager.supplierCache.get(name.toLowerCase());
  }

  @SuppressWarnings("unchecked")
  private <T> LogEntry<T> addToEntryArray(int i, String name, Data<T> data, String metaData) {
    if (categoryLogEntries[i] != null && categoryLogEntries[i].getData() != null) {
      boolean groupIsSignal = categoryLogEntries[i].getData().getSignals() != null;
      boolean newIsSignal = data.getSignals() != null;

      if (groupIsSignal != newIsSignal) {
          LogManager.log("Log Type Mismatch in '" + name + "'. Creating separate entry.", AlertType.kWarning);
          return add(name, data, LogLevel.LOG_ONLY, metaData, false);
      }
    }
  
    int subIndex;
    int dataIndex = 0;
    
    if (categoryLogEntries[i] == null) {
        categoryLogEntries[i] = new LogEntry<>(name, data, getLogLevelFromIndex(i), metaData);
        subIndex = 1;
        dataIndex = 0;
    } else {
        String[] parts = getCachedSplit(categoryLogEntries[i].getName());
        subIndex = parts.length + 1;

        if (categoryLogEntries[i].getData().getSignals() != null) {
          dataIndex = categoryLogEntries[i].getData().getSignals().length;
        } else if (categoryLogEntries[i].getData().getSuppliers() != null) {
          dataIndex = categoryLogEntries[i].getData().getSuppliers().length;
        }

        try {
            ((LogEntry<T>) categoryLogEntries[i]).addData(name, data, metaData);
        } catch (Exception e) {
            LogManager.log("Error combining log entries: " + e.getMessage(), AlertType.kError);
        }
    }

    int dataLength = (data.getSignals() != null) ? data.getSignals().length : data.getSuppliers().length;
    entryLocationMap.put(name, new Integer[]{i, subIndex, dataIndex, dataLength});
    
    return (LogEntry<T>) categoryLogEntries[i];
  }

  private LogLevel getLogLevelFromIndex(int i) {
    if (i <= 3) return LogLevel.LOG_ONLY_NOT_IN_COMP;
    if (i <= 7) return LogLevel.LOG_ONLY;
    if (i <= 11) return LogLevel.LOG_AND_NT_NOT_IN_COMP;
    return LogLevel.LOG_AND_NT;
  }

  private String[] getCachedSplit(String name) {
    return nameSplitCache.computeIfAbsent(name, k -> k.split(" \\| "));
  }

  private int getCategoryIndex(Data<?> data, LogLevel logLevel, Boolean isSeperated) {
    boolean isSignal = data.getSignals() != null;
    boolean isSupplier = data.getSuppliers() != null;
    boolean isDouble = data.isDouble();
    boolean isBoolean = data.isBoolean();
    
    if (!(isDouble || isBoolean) || !(isSignal || isSupplier) || isSeperated) {
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
          if (location[0] == categoryIndex && location[1] > removedSubIndex) {
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
}