package frc.demacia.utils.Log;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import frc.demacia.utils.Data;

/**
 * Fluent builder for creating log entries with various options.
 */
public class LogEntryBuilder<T> {

    /**
     * Enum representing LogLevel.
     */
    public static enum LogLevel { LOG_ONLY_NOT_IN_COMP, LOG_ONLY, LOG_AND_NT_NOT_IN_COMP, LOG_AND_NT} 

    private String name;
    private LogLevel logLevel = LogLevel.LOG_ONLY_NOT_IN_COMP;
    private String metadata = "";
    private BiConsumer<T[], Long> consumer = null;
    private boolean isSeparated = false;
    private Data<T> data;
    
    @SafeVarargs
    LogEntryBuilder(String name, StatusSignal<T>... statusSignals) {
        this.name = name;
        this.data = new Data<>(statusSignals);
    }
    
    @SafeVarargs
    LogEntryBuilder(String name, Supplier<T>... suppliers) {
        this.name = name;
        this.data = new Data<>(suppliers);
    }
    
    public LogEntryBuilder<T> withLogLevel(LogLevel level) {
        this.logLevel = level;
        return this;
    }
    
    public LogEntryBuilder<T> withMetaData(String metaData) {
        this.metadata = metaData;
        return this;
    }
    
    public LogEntryBuilder<T> withIsMotor() {
        this.metadata = "motor";
        return this;
    }
    
    public LogEntryBuilder<T> withConsumer(BiConsumer<T[], Long> consumer) {
        this.consumer = consumer;
        this.isSeparated = true; 
        return this;
    }

    public LogEntryBuilder<T> withIsSeparated(boolean isSeparated) {
        if (this.consumer == null) {
            this.isSeparated = isSeparated;
        }
        return this;
    }
    
    public LogEntry<T> build() {
        if (name == null || name.trim().isEmpty()) {
            LogManager.log("Log entry name cannot be null or empty");
            return null;
        }
        if (logLevel == null) {
            LogManager.log("Log level cannot be null");
            return null;
        }
        
        LogEntry<T> entry = LogManager.add(name, data, logLevel, metadata, isSeparated);
        
        if (consumer != null) {
            entry.setConsumer(consumer);
        }
        return entry;
    }
}