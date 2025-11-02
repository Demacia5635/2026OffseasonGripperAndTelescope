package frc.demacia.utils.Log;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusSignal;

import frc.demacia.utils.Data;

public class LogEntryBuilder<T> implements AutoCloseable{
    private String name;
    private int logLevel = 2;
    private String metaData = "";
    private double precision = 0.0;
    private int skipCycles = 1;
    private BiConsumer<T[], Long> consumer = null;

    private Data<T> data;

    private boolean built = false;
    
    LogEntryBuilder(String name, StatusSignal<T> ... statusSignals) {
        this.name = name;
        data = new Data<>(statusSignals);
    }
    
    LogEntryBuilder(String name, Supplier<T> ... suppliers) {
        this.name = name;
        data = new Data<>(suppliers);
    }
    
    public LogEntryBuilder<T> logLevel(int level) {
        this.logLevel = level;
        return this;
    }
    
    public LogEntryBuilder<T> metaData(String metaData) {
        this.metaData = metaData;
        return this;
    }

    
    
    public LogEntryBuilder<T> isMotor() {
        this.metaData = "motor";
        return this;
    }
    
    public LogEntryBuilder<T> precision(double precision) {
        this.precision = precision;
        return this;
    }
    
    public LogEntryBuilder<T> skipCycles(int cycles) {
        this.skipCycles = cycles;
        return this;
    }
    
    public LogEntryBuilder<T> consumer(BiConsumer<T[], Long> consumer) {
        this.consumer = consumer;
        return this;
    }
    
    public LogEntry2<T> build() {
        built = true;
        LogEntry2<T> entry = LogManager2.logManager.add(name, data, logLevel, metaData);
        if (precision != 0.0) {
            entry.setPrecision(precision);
        }
        if (skipCycles != 1) {
            entry.setSkipCycles(skipCycles);
        }
        if (consumer != null) {
            entry.setConsumer(consumer);
        }
        return entry;
    }
    
    @Override
    public void close() {
        if (!built) {
            build();
        }
    }
}