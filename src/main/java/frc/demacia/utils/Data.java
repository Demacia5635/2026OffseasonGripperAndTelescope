package frc.demacia.utils;

import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class Data<T> {
    
    private static ArrayList<WeakReference<Data<?>>> signals = new ArrayList<>();

    private StatusSignal<T>[] signal;
    private Supplier<T>[] supplier;
    private Supplier<T>[] oldSupplier;
    private T[] currentValues;
    private int length;

    private boolean isDouble = false;
    private boolean isBoolean = false;
    private boolean isArray = false;

    private boolean changed = true;

    private double[] cachedDoubleArray;
    private float[] cachedFloatArray;
    private boolean[] cachedBooleanArray;
    private String[] cachedStringArray;

    @SuppressWarnings("unchecked")
    public Data(StatusSignal<T>... signal){
        this.signal = signal;
        length = signal.length;
        currentValues = (T[]) new Object[length];
        
        if (length > 1) isArray = true;

        detectTypeFromSignal();
        allocateCachedArrays();

        refresh();
        register();
    }
    
    @SuppressWarnings("unchecked")
    public Data(Supplier<T>... supplier){
        this.supplier = supplier;
        this.oldSupplier = supplier;

        T value = supplier.length > 0 ? supplier[0].get() : null;

        if (value == null){
            length = 0;
            currentValues = (T[]) new Object[0];
            register();
            return;
        }

        length = supplier.length;
        currentValues = (T[]) new Object[length];

        if (value.getClass().isArray()) {
            handleArraySupplier(value, supplier);
        } else {
            handleScalarSupplier(value, supplier);
        }

        allocateCachedArrays();
        refresh();
        register();
    }

    private void register() {
        synchronized (signals) {
            signals.add(new WeakReference<>(this));
        }
    }

    private void detectTypeFromSignal() {
        if (length == 0) return;
        try {
            if (signal[0].getValue() instanceof Boolean) {
                isBoolean = true;
            } else {
                signal[0].getValueAsDouble(); 
                isDouble = true;
            }
        } catch (Exception e) {
            if (signal[0].getValue() instanceof Boolean) isBoolean = true;
        }
    }

    private void handleArraySupplier(T value, Supplier<T>[] supplier) {
        isArray = true;
        detectArrayType(value);
    }

    private void detectArrayType(T value) {
        if (java.lang.reflect.Array.getLength(value) == 0) return;
        Object first = java.lang.reflect.Array.get(value, 0);
        if (first instanceof Number) isDouble = true;
        else if (first instanceof Boolean) isBoolean = true;
    }

    @SuppressWarnings("unchecked")
    private void handleScalarSupplier(T value, Supplier<T>[] supplier) {
        if (value instanceof Number) isDouble = true;
        else if (value instanceof Boolean) isBoolean = true;
        
        if (length > 1){
            isArray = true;
            this.oldSupplier = Arrays.copyOf(supplier, supplier.length);
            
            this.supplier = new Supplier[] {
                () -> {
                    T[] freshValues = (T[]) new Object[this.oldSupplier.length];
                    for (int i = 0; i < this.oldSupplier.length; i++){
                        freshValues[i] = this.oldSupplier[i].get();
                    }
                    return freshValues;
                }
            };
            length = 1;
        }
    }

    private void allocateCachedArrays() {
        int size = getEffectiveLength();
        if (size > 0) {
            if (isDouble) {
                cachedDoubleArray = new double[size];
                cachedFloatArray = new float[size];
            } else if (isBoolean) {
                cachedBooleanArray = new boolean[size];
            } else {
                cachedStringArray = new String[size];
            }
        }
    }

    private int getEffectiveLength() {
        if (signal != null) return length;
        if (currentValues == null || currentValues.length == 0) return 0;
        
        if (isArray) {
            int total = 0;
            for(T val : currentValues) {
                if (val != null && val.getClass().isArray()) {
                    total += java.lang.reflect.Array.getLength(val);
                }
            }
            return total > 0 ? total : 0;
        }
        return length;
    }

    public void refresh() {
        changed = false;

        if (signal != null) {
            StatusCode st = StatusSignal.refreshAll(signal);
            if(st.isOK()) {
                if (isDouble) {
                    if (cachedDoubleArray == null) return; 
                    for (int i = 0; i < length; i++) {
                        double newVal = signal[i].getValueAsDouble();
                        if (cachedDoubleArray[i] != newVal) {
                            changed = true;
                            cachedDoubleArray[i] = newVal;
                        }
                    }
                } else {
                    for (int i = 0; i < length; i++){
                        T newVal = signal[i].getValue();
                        
                        if (isBoolean) {
                             if (cachedBooleanArray == null) return;
                             boolean bVal = (Boolean) newVal;
                             if (cachedBooleanArray[i] != bVal) {
                                 changed = true;
                                 cachedBooleanArray[i] = bVal;
                             }
                        } else {
                             if (cachedStringArray == null) return;
                             String sVal = newVal.toString();
                             if (!Objects.equals(cachedStringArray[i], sVal)) {
                                 changed = true;
                                 cachedStringArray[i] = sVal;
                             }
                        }
                    }
                }
            }
        } else {
            boolean anyChanged = false;
            for (int i = 0; i < length; i++){
                T newVal = supplier[i].get();
                if (!Objects.deepEquals(currentValues[i], newVal)) {
                    anyChanged = true;
                    currentValues[i] = newVal;
                }
            }
            
            if (anyChanged) {
                changed = true;
                if (isDouble) toDoubleArray(currentValues);
                else if (isBoolean) toBooleanArray(currentValues);
                else toStringArray(currentValues);
            }
        }
    }

    public static void refreshAll() {
        synchronized (signals) {
            Iterator<WeakReference<Data<?>>> iterator = signals.iterator();
            while (iterator.hasNext()) {
                Data<?> data = iterator.next().get();
                if (data == null) iterator.remove();
                else data.refresh();
            }
        }
    }

    public boolean hasChanged() { return changed; }

    public Double getDouble() {
        if (!isDouble || length == 0) return null;
        if (cachedDoubleArray != null && cachedDoubleArray.length > 0) return cachedDoubleArray[0];
        if (signal != null) return signal[0].getValueAsDouble();
        return toDouble(currentValues[0]);
    }

    public double[] getDoubleArray() {
        if (!isDouble || length == 0) return null;
        return cachedDoubleArray;
    }

    public Float getFloat() {
        if (!isDouble || length == 0) return null;
        if (cachedDoubleArray != null && cachedDoubleArray.length > 0) return (float) cachedDoubleArray[0];
        if (signal != null) return (float) signal[0].getValueAsDouble();
        Double d = toDouble(currentValues[0]);
        return d != null ? d.floatValue() : null;
    }

    public float[] getFloatArray() {
        if (!isDouble || length == 0) return null;
        if (cachedDoubleArray != null) {
            if (cachedFloatArray == null || cachedFloatArray.length != cachedDoubleArray.length) {
                cachedFloatArray = new float[cachedDoubleArray.length];
            }
            for(int i=0; i<cachedDoubleArray.length; i++) {
                cachedFloatArray[i] = (float)cachedDoubleArray[i];
            }
            return cachedFloatArray;
        }
        return toFloatArray(currentValues);
    }

    public Boolean getBoolean() {
        if (!isBoolean || length == 0) return null;
        if (signal != null) return (Boolean) signal[0].getValue();
        return (Boolean) currentValues[0];
    }

    public boolean[] getBooleanArray() {
        if (!isBoolean || length == 0) return null;
        return cachedBooleanArray;
    }

    public String getString() {
        if (isDouble || isBoolean || length == 0) return "";
        if (signal != null && signal.length > 0) return signal[0].getValue().toString();
        return currentValues[0] != null ? currentValues[0].toString() : "";
    }

    public String[] getStringArray() {
        if (isDouble || isBoolean || length == 0) return null;
        return cachedStringArray;
    }

    public T getValue() { 
        if (signal != null && length > 0) return signal[0].getValue();
        return length == 0 ? null : currentValues[0]; 
    }
    
    public T[] getValueArray() { return length == 0 ? null : currentValues; }
    
    public StatusSignal<T> getSignal() { return (signal != null && signal.length > 0) ? signal[0] : null; }
    public StatusSignal<T>[] getSignals() {return signal;}
    public Supplier<T> getSupplier() {
        if (signal != null && signal.length > 0) return () -> signal[0].getValue();
        return (oldSupplier != null && oldSupplier.length > 0) ? oldSupplier[0] : null;
    }
    public Supplier<T>[] getSuppliers() {
        if (signal != null) {
            @SuppressWarnings("unchecked")
            Supplier<T>[] sigSuppliers = new Supplier[signal.length];
            for (int i = 0; i < signal.length; i++) {
                final int index = i;
                sigSuppliers[i] = () -> signal[index].getValue();
            }
            return sigSuppliers;
        }
        return oldSupplier;
    }

    public long getTime() {
        if (signal != null) return (long) (signal[0].getTimestamp().getTime() * 1000);
        return 0;
    }

    public boolean isDouble() { return isDouble; }
    public boolean isBoolean() { return isBoolean; }
    public boolean isArray() { return isArray; }

    @SuppressWarnings("unchecked")
    public void expandWithSignals(StatusSignal<T>[] newSignals) {
        if (signal == null || newSignals == null || newSignals.length == 0) return;
        int oldLength = signal.length;
        int newLength = oldLength + newSignals.length;
        StatusSignal<T>[] expandedSignals = new StatusSignal[newLength];
        System.arraycopy(signal, 0, expandedSignals, 0, oldLength);
        System.arraycopy(newSignals, 0, expandedSignals, oldLength, newSignals.length);
        signal = expandedSignals;
        length = newLength;
        if (length > 1) isArray = true;
        allocateCachedArrays();
        refresh();
    }

    @SuppressWarnings("unchecked")
    public void expandWithSuppliers(Supplier<T>[] newSuppliers) {
        if (supplier == null || newSuppliers == null || newSuppliers.length == 0) return;
        Supplier<?>[] existingSuppliers = getSuppliers();
        Supplier<T>[] combined = new Supplier[existingSuppliers.length + newSuppliers.length];
        System.arraycopy(existingSuppliers, 0, combined, 0, existingSuppliers.length);
        System.arraycopy(newSuppliers, 0, combined, existingSuppliers.length, newSuppliers.length);
        
        T value = combined[0].get();
        if (value == null) {
            supplier = combined; oldSupplier = combined; length = 0;
            currentValues = (T[]) new Object[0]; return;
        }
        if (value.getClass().isArray()) {
            handleArraySupplier(value, combined);
            supplier = combined; oldSupplier = combined; length = combined.length;
        } else {
            handleScalarSupplier(value, combined);
        }
        currentValues = (T[]) new Object[length];
        allocateCachedArrays();
        refresh();
    }

    @SuppressWarnings("unchecked")
    public void removeSignalRange(int startIndex, int count) {
        if (signal == null || signal.length == 0 || count <= 0) return;
        int newLength = length - count;
        StatusSignal<T>[] newSignals = new StatusSignal[newLength];
        if (startIndex > 0) System.arraycopy(signal, 0, newSignals, 0, startIndex);
        if (startIndex + count < length) System.arraycopy(signal, startIndex + count, newSignals, startIndex, length - startIndex - count);
        signal = newSignals; length = newLength; isArray = length > 1;
        allocateCachedArrays(); refresh();
    }
    public void removeSignal(int startIndex){ removeSignalRange(startIndex, 1); }

    @SuppressWarnings("unchecked")
    public void removeSupplierRange(int startIndex, int count) {
        if (oldSupplier == null || oldSupplier.length == 0 || count <= 0) return;
        count = Math.min(count, oldSupplier.length - startIndex);
        int newLength = oldSupplier.length - count;
        Supplier<T>[] newOldSuppliers = new Supplier[newLength];
        System.arraycopy(oldSupplier, 0, newOldSuppliers, 0, startIndex);
        if (startIndex + count < oldSupplier.length) System.arraycopy(oldSupplier, startIndex + count, newOldSuppliers, startIndex, oldSupplier.length - startIndex - count);
        oldSupplier = newOldSuppliers;
        T value = newOldSuppliers.length > 0 ? newOldSuppliers[0].get() : null;
        if (value == null) {
            supplier = newOldSuppliers; length = 0; currentValues = (T[]) new Object[0]; isArray = false; return;
        }
        if (value.getClass().isArray()) {
            handleArraySupplier(value, newOldSuppliers); supplier = newOldSuppliers; length = newOldSuppliers.length;
        } else {
            handleScalarSupplier(value, newOldSuppliers);
        }
        currentValues = (T[]) new Object[length]; allocateCachedArrays(); refresh();
    }
    public void removeSupplier(int startIndex){ removeSupplierRange(startIndex, 1); }

    private double[] toDoubleArray(T[] value){
        if (value == null) return null;
        int totalSize = 0;
        if (isArray) {
            for (T val : value) if (val != null) totalSize += java.lang.reflect.Array.getLength(val);
        } else {
            totalSize = length;
        }

        if (cachedDoubleArray == null || cachedDoubleArray.length != totalSize) cachedDoubleArray = new double[totalSize];

        if (!isArray) {
            for (int i = 0; i < length; i++) {
                if (value[i] instanceof Number) cachedDoubleArray[i] = ((Number) value[i]).doubleValue();
                else cachedDoubleArray[i] = 0.0;
            }
        } else {
            int offset = 0;
            for (T val : value) {
                if (val != null) {
                    int len = java.lang.reflect.Array.getLength(val);
                    for (int i = 0; i < len; i++) {
                        Object elem = java.lang.reflect.Array.get(val, i);
                        cachedDoubleArray[offset++] = (elem != null) ? ((Number) elem).doubleValue() : 0.0;
                    }
                }
            }
        }
        return cachedDoubleArray;
    }

    private Double toDouble(T value){
        if (value == null) return null;
        if (isArray) {
            Object first = java.lang.reflect.Array.get(value, 0);
            return (first != null) ? ((Number) first).doubleValue() : null;
        }
        return ((Number) value).doubleValue();
    }

    private float[] toFloatArray(T[] value){
        if (value == null) return null;
        int totalSize = 0;
        if (isArray) {
            for (T val : value) if (val != null) totalSize += java.lang.reflect.Array.getLength(val);
        } else {
            totalSize = length;
        }

        if (cachedFloatArray == null || cachedFloatArray.length != totalSize) cachedFloatArray = new float[totalSize];

        if (!isArray) {
            for (int i = 0; i < length; i++) cachedFloatArray[i] = (value[i] != null) ? ((Number) value[i]).floatValue() : 0f;
        } else {
            int offset = 0;
            for (T val : value) {
                if (val != null) {
                    int len = java.lang.reflect.Array.getLength(val);
                    for (int i = 0; i < len; i++) {
                        Object elem = java.lang.reflect.Array.get(val, i);
                        cachedFloatArray[offset++] = (elem != null) ? ((Number) elem).floatValue() : 0f;
                    }
                }
            }
        }
        return cachedFloatArray;
    }

    private boolean[] toBooleanArray(T[] value){
        if (value == null) return null;
        int totalSize = 0;
        if (isArray) {
            for (T val : value) if (val != null) totalSize += java.lang.reflect.Array.getLength(val);
        } else {
            totalSize = length;
        }

        if (cachedBooleanArray == null || cachedBooleanArray.length != totalSize) cachedBooleanArray = new boolean[totalSize];
        
        if (!isArray) {
            for (int i = 0; i < length; i++) cachedBooleanArray[i] = (value[i] != null) && (Boolean) value[i];
        } else {
            int offset = 0;
            for (T val : value) {
                if (val != null) {
                    int len = java.lang.reflect.Array.getLength(val);
                    for (int i = 0; i < len; i++) {
                        Object elem = java.lang.reflect.Array.get(val, i);
                        cachedBooleanArray[offset++] = (elem != null) && (Boolean) elem;
                    }
                }
            }
        }
        return cachedBooleanArray;
    }
    
    private String[] toStringArray(T[] value){
        if (value == null) return null;
        int totalSize = 0;
        if (isArray) {
            for (T val : value) if (val != null) totalSize += java.lang.reflect.Array.getLength(val);
        } else {
            totalSize = length;
        }

        if (cachedStringArray == null || cachedStringArray.length != totalSize) cachedStringArray = new String[totalSize];
        
        if (!isArray) {
            for (int i = 0; i < length; i++) cachedStringArray[i] = (value[i] != null) ? value[i].toString() : null;
        } else {
            int offset = 0;
            for (T val : value) {
                if (val != null) {
                    int len = java.lang.reflect.Array.getLength(val);
                    for (int i = 0; i < len; i++) {
                        Object elem = java.lang.reflect.Array.get(val, i);
                        cachedStringArray[offset++] = (elem != null) ? elem.toString() : null;
                    }
                }
            }
        }
        return cachedStringArray;
    }

    public void cleanup() {
        signal = null; supplier = null; currentValues = null; cachedDoubleArray = null;
    }

    public static void clearAllSignals() { signals.clear(); }
}