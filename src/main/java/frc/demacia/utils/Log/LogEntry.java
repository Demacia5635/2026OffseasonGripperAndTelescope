// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.Log;

import java.util.function.BiConsumer;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.util.datalog.FloatLogEntry;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.demacia.utils.Data;
import frc.demacia.utils.DemaciaUtils;
import frc.demacia.utils.Log.LogEntryBuilder.LogLevel;

public class LogEntry<T> {
    private DataLogEntry entry;
    private Data<T> data;
    private BiConsumer<T[], Long> consumer = null;

    private String name;
    private String metaData;
    private Publisher ntPublisher;
    private LogLevel logLevel;

    private BiConsumer<Long, Data<T>> logStrategy;
    private BiConsumer<Data<T>, Publisher> ntStrategy;
    /*
        * Constructor with the suppliers and boolean if add to network table
    */
    LogEntry(String name, Data<T> data, LogLevel logLevel, String metaData) {
        this.name = name;
        this.logLevel = logLevel;
        this.data = data;
        this.metaData = metaData;

        initializeLogging();
    }

    private void initializeLogging() {
        if (ntPublisher != null) ntPublisher.close();
        if (entry != null) entry.finish();

        createLogEntry(LogManager.log, name, metaData);

        if (logLevel == LogLevel.LOG_AND_NT || (logLevel == LogLevel.LOG_AND_NT_NOT_IN_COMP && !DemaciaUtils.getIsComp())) {
            createPublisher(LogManager.table, name);
        } else {
            ntPublisher = null;
            ntStrategy = null;
        }
    }

    void log() {
        if (!data.hasChanged()) {
            return;
        }

        long time = data.getTime();

        if (logStrategy != null) {
            logStrategy.accept(time, data);
        }

        if (ntPublisher != null && ntStrategy != null) {
            ntStrategy.accept(data, ntPublisher);
        }
        
        if (consumer != null) {
            consumer.accept(data.getValueArray(), time);
        } 
    }

    public String getName(){
        return name;
    }

    public Data<T> getData(){
        return data;
    }

    public String getMetaData(){
        return metaData;
    }

    public LogLevel getLogLevel(){
        return logLevel;
    }

    public void setConsumer(BiConsumer<T[], Long> consumer) {
        this.consumer = consumer;
    }

    public BiConsumer<T[], Long> getConsumer(){
        return consumer;
    }

    public void removeInComp() {
        if (logLevel == LogLevel.LOG_AND_NT_NOT_IN_COMP && ntPublisher != null) {
            ntPublisher.close();
        }
    }

    private void createLogEntry(DataLog log, String name, String metaData) {
        boolean isFloat = data.isDouble();
        boolean isBoolean = data.isBoolean();
        boolean isArray = data.isArray();

        if (isArray) {
            if (isFloat){
                entry = new FloatArrayLogEntry(log, name, metaData);
                logStrategy = (time, d) -> ((FloatArrayLogEntry) entry).append(d.getFloatArray(), time);
            } else if (isBoolean){
                entry = new BooleanArrayLogEntry(log, name, metaData);
                logStrategy = (time, d) -> ((BooleanArrayLogEntry) entry).append(d.getBooleanArray(), time);
            } else{
                entry = new StringArrayLogEntry(log, name, metaData);
                logStrategy = (time, d) -> ((StringArrayLogEntry) entry).append(d.getStringArray(), time);
            }
        } else {
            if (isFloat){
                entry = new FloatLogEntry(log, name, metaData);
                logStrategy = (time, d) -> ((FloatLogEntry) entry).append(d.getFloat(), time);
            } else if (isBoolean){
                entry = new BooleanLogEntry(log, name, metaData);
                logStrategy = (time, d) -> ((BooleanLogEntry) entry).append(d.getBoolean(), time);
            } else{
                entry = new StringLogEntry(log, name, metaData);
                logStrategy = (time, d) -> ((StringLogEntry) entry).append(d.getString(), time);
            }
        }
    }

    private void createPublisher(NetworkTable table, String name) {
        boolean isFloat = data.isDouble();
        boolean isBoolean = data.isBoolean();
        boolean isArray = data.isArray();

        if (isArray) {
            if (isFloat){
                ntPublisher = table.getFloatArrayTopic(name).publish();
                ntStrategy = (d, p) -> ((FloatArrayPublisher) p).set(d.getFloatArray());
            } else if (isBoolean){
                ntPublisher = table.getBooleanArrayTopic(name).publish();
                ntStrategy = (d, p) -> ((BooleanArrayPublisher) p).set(d.getBooleanArray());
            } else{
                ntPublisher = table.getStringArrayTopic(name).publish();
                ntStrategy = (d, p) -> ((StringArrayPublisher) p).set(d.getStringArray());
            }
        } else {
            if (isFloat){
                ntPublisher = table.getFloatTopic(name).publish();
                ntStrategy = (d, p) -> ((FloatPublisher) p).set(d.getFloat());
            } else if (isBoolean){
                ntPublisher = table.getBooleanTopic(name).publish();
                ntStrategy = (d, p) -> ((BooleanPublisher) p).set(d.getBoolean());
            } else{
                ntPublisher = table.getStringTopic(name).publish();
                ntStrategy = (d, p) -> ((StringPublisher) p).set(d.getString());
            }
        }
    }

    public void addData(String name, Data<T> data, String metaData){
        this.name = this.name + " | " + name;
        this.metaData = this.metaData + " | " + metaData;
        if (this.data.getSignals() != null){
            this.data.expandWithSignals(data.getSignals());
        } else {
            this.data.expandWithSuppliers(data.getSuppliers());
        }

        initializeLogging();
    }

    public void removeData(int nameIndex, int dataIndex, int count) {
        int actualIndex = nameIndex - 1;
        String[] parts = name.split(" \\| ");

        if (actualIndex >= parts.length || actualIndex < 0) {
            LogManager.log("removeData: nameIndex out of range: " + nameIndex + " (parts length: " + parts.length + ")", AlertType.kWarning);
            return;
        }

        StringBuilder newName = new StringBuilder();
        for (int i = 0; i < parts.length; i++) {
            if (i != actualIndex) {
                if (newName.length() > 0) newName.append(" | ");
                newName.append(parts[i]);
            }
        }

        this.name = newName.toString();

        if (data != null) {
            try {
                if (data.getSignals() != null) {
                    data.removeSignalRange(dataIndex, count);
                } else if (data.getSuppliers() != null) {
                    data.removeSupplierRange(dataIndex, count);
                }
            } catch (Exception e) {
                LogManager.log("removeData: failed to remove from data: " + e.getMessage(), AlertType.kError);
            }
        }

        if (this.name.isEmpty()) {
            if (ntPublisher != null) ntPublisher.close();
            if (entry != null) entry.finish();
            ntPublisher = null;
            entry = null;
            data = null;
            return;
        }
        
        initializeLogging();
    }
}