package frc.demacia.utils;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

/**
 * Generic data wrapper for telemetry and logging.
 * 
 * <p>Handles both CTRE StatusSignals and generic Suppliers with change detection
 * and type conversion for logging.</p>
 * 
 * <p><b>Features:</b></p>
 * <ul>
 *   <li>Automatic type detection (double, boolean, string)</li>
 *   <li>Change detection with configurable precision</li>
 *   <li>Array and scalar support</li>
 *   <li>Timestamp synchronization for CTRE signals</li>
 *   <li>Dynamic expansion for grouped logging</li>
 * </ul>
 * 
 * <p><b>Example Usage:</b></p>
 * <pre>
 * // From CTRE StatusSignals
 * Data<Double> motorData = new Data<>(
 *     motor.getPosition(),
 *     motor.getVelocity(),
 *     motor.getMotorVoltage()
 * );
 * 
 * // From Suppliers
 * Data<Double> sensorData = new Data<>(
 *     () -> sensor1.getValue(),
 *     () -> sensor2.getValue()
 * );
 * 
 * // Check if data changed (for efficient logging)
 * if (motorData.hasChanged()) {
 *     log(motorData.getDoubleArray());
 * }
 * </pre>
 * 
 * @param <T> The data type (Double, Boolean, String, or arrays)
 */
public class Data<T> {
    
    private static ArrayList<Data<?>> signals = new ArrayList<>();

    private StatusSignal<T>[] signal;
    private Supplier<T>[] supplier;
    private Supplier<T>[] oldSupplier;
    private T[] currentValues;
    private int length;

    private boolean isDouble = false;
    private boolean isBoolean = false;
    private boolean isArray = false;

    private boolean changed = true;

    // Cache for converted arrays - reuse instead of recreating
    private double[] cachedDoubleArray;
    private float[] cachedFloatArray;
    private boolean[] cachedBooleanArray;
    private String[] cachedStringArray;

    /**
     * Constructor from CTRE StatusSignals.
     * 
     * <p>Provides timestamp synchronization and efficient bulk refresh.</p>
     * 
     * @param signal One or more StatusSignal objects
     */
    @SuppressWarnings("unchecked")
    public Data(StatusSignal<T>... signal){
        this.signal = signal;

        signals.add(this);
        
        length = signal.length;

        currentValues = (T[]) new Object[length];
        
        refresh();

        if (length > 1){
            isArray = true;
        }

        detectTypeFromSignal();
        allocateCachedArrays();
    }
    
    /**
     * Constructor from Suppliers.
     * 
     * <p>Calls suppliers each cycle to get fresh data.</p>
     * 
     * @param supplier One or more Supplier functions
     */
    @SuppressWarnings("unchecked")
    public Data(Supplier<T>... supplier){
        this.supplier = supplier;
        this.oldSupplier = supplier;

        signals.add(this);

        T value = supplier[0].get();

        if (value == null){
            length = 0;
            currentValues = (T[]) new Object[0];
            return;
        }

        length = supplier.length;
        currentValues = (T[]) new Object[length];
        
        refresh();

        if (value.getClass().isArray()) {
            handleArraySupplier(value, supplier);
        } else {
            handleScalarSupplier(value, supplier);
        }

        allocateCachedArrays();
    }

    // Separate type detection to avoid repeated checks
    private void detectTypeFromSignal() {
        try {
            signal[0].getValueAsDouble();
            isDouble = true;
        } catch (Exception e) {
            isBoolean = signal[0].getValue() instanceof Boolean;
        }
    }

    @SuppressWarnings("unchecked")
    private void handleArraySupplier(T value, Supplier<T>[] supplier) {
        isArray = true;
        int arrayLength = java.lang.reflect.Array.getLength(value);
        
        oldSupplier = new Supplier[arrayLength];
        final T finalValue = value;
        for (int i = 0; i < arrayLength; i++){
            final int index = i;
            oldSupplier[i] = () -> (T) java.lang.reflect.Array.get(finalValue, index);
        }
        
        detectArrayType(value);
    }

    private void detectArrayType(T value) {
        try {
            Object first = java.lang.reflect.Array.get(value, 0);
            ((Number) first).doubleValue();
            isDouble = true;
        } catch (Exception e) {
            isBoolean = (value instanceof boolean[] || value instanceof Boolean[]);
        }
    }

    @SuppressWarnings("unchecked")
    private void handleScalarSupplier(T value, Supplier<T>[] supplier) {
        try {
            ((Number) value).doubleValue();
            isDouble = true;
        } catch (Exception e) {
            isBoolean = (value instanceof Boolean);
        }
        
        if (length > 1){
            isArray = true;
            this.oldSupplier = Arrays.copyOf(supplier, supplier.length);
            T[] valueArray = (T[]) new Object[length];
            for (int i = 0; i < length; i++){
                valueArray[i] = supplier[i].get();
            }
            this.supplier = new Supplier[] {() -> valueArray};
            length = 1;
        }
    }

    // Pre-allocate cached arrays
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
        if (currentValues == null || currentValues.length == 0) return 0;
        if (isArray && currentValues[0] != null && currentValues[0].getClass().isArray()) {
            return java.lang.reflect.Array.getLength(currentValues[0]);
        }
        return length;
    }

    /**
     * Expands this Data object with additional signals.
     * 
     * <p>Used by LogManager to group related signals for efficient logging.</p>
     * 
     * @param newSignals Signals to add
     */
    @SuppressWarnings("unchecked")
    public void expandWithSignals(StatusSignal<T>[] newSignals) {
        if (signal == null || newSignals == null || newSignals.length == 0) {
            return;
        }
        
        int oldLength = signal.length;
        int newLength = oldLength + newSignals.length;
        
        StatusSignal<T>[] expandedSignals = new StatusSignal[newLength];
        System.arraycopy(signal, 0, expandedSignals, 0, oldLength);
        System.arraycopy(newSignals, 0, expandedSignals, oldLength, newSignals.length);
        signal = expandedSignals;
        
        T[] expandedCurrent = (T[]) new Object[newLength];
        
        if (currentValues != null) {
            System.arraycopy(currentValues, 0, expandedCurrent, 0, Math.min(oldLength, currentValues.length));
        }
        
        currentValues = expandedCurrent;
        length = newLength;
        
        if (length > 1) {
            isArray = true;
        }
        
        allocateCachedArrays();
        refresh();
    }
    

    /**
     * Expands this Data object with additional suppliers.
     * 
     * @param newSuppliers Suppliers to add
     */
    @SuppressWarnings("unchecked")
    public void expandWithSuppliers(Supplier<T>[] newSuppliers) {
        if (supplier == null || newSuppliers == null || newSuppliers.length == 0) {
            return;
        }
    
        Supplier<?>[] existingSuppliers = getSuppliers();
        
        Supplier<T>[] combined = new Supplier[existingSuppliers.length + newSuppliers.length];
        System.arraycopy(existingSuppliers, 0, combined, 0, existingSuppliers.length);
        System.arraycopy(newSuppliers, 0, combined, existingSuppliers.length, newSuppliers.length);
        
        T value = combined[0].get();
        
        if (value == null) {
            supplier = combined;
            oldSupplier = combined;
            length = 0;
            currentValues = (T[]) new Object[0];
            return;
        }
        
        if (value.getClass().isArray()) {
            handleArraySupplier(value, combined);
        } else {
            handleScalarSupplier(value, combined);
        }
        
        currentValues = (T[]) new Object[length];
        allocateCachedArrays();
        refresh();
    }

    /**
     * Removes a range of signals.
     * 
     * <p>Used when log entries are removed.</p>
     * 
     * @param startIndex First index to remove
     * @param count Number of signals to remove
     */
    @SuppressWarnings("unchecked")
    public void removeSignalRange(int startIndex, int count) {
        if (signal == null || signal.length == 0 || count <= 0) return;

        int newLength = length - count;
        StatusSignal<T>[] newSignals = new StatusSignal[newLength];
        
        if (startIndex > 0) {
            System.arraycopy(signal, 0, newSignals, 0, startIndex);
        }
        if (startIndex + count < length) {
            System.arraycopy(signal, startIndex + count, newSignals, startIndex, length - startIndex - count);
        }

        signal = newSignals;
        length = newLength;
        
        currentValues = (T[]) new Object[length];

        isArray = length > 1;

        allocateCachedArrays();
        refresh();
    }
    

    public void removeSignal(int startIndex){
        removeSignalRange(startIndex, 1);
    }


    @SuppressWarnings("unchecked")
    public void removeSupplierRange(int startIndex, int count) {
        if (oldSupplier == null || oldSupplier.length == 0 || count <= 0) return;
        if (startIndex < 0 || startIndex >= oldSupplier.length) return;
        
        count = Math.min(count, oldSupplier.length - startIndex);
        
        int newLength = oldSupplier.length - count;
        Supplier<T>[] newOldSuppliers = new Supplier[newLength];
        
        System.arraycopy(oldSupplier, 0, newOldSuppliers, 0, startIndex);
        
        if (startIndex + count < oldSupplier.length) {
            System.arraycopy(oldSupplier, startIndex + count, newOldSuppliers, 
                            startIndex, oldSupplier.length - startIndex - count);
        }
        
        oldSupplier = newOldSuppliers;
        
        T value = newOldSuppliers.length > 0 ? newOldSuppliers[0].get() : null;
        
        if (value == null) {
            supplier = newOldSuppliers;
            length = 0;
            currentValues = (T[]) new Object[0];
            isArray = false;
            return;
        }
        
        if (value.getClass().isArray()) {
            handleArraySupplier(value, new Supplier[]{newOldSuppliers[0]});
        } else {
            handleScalarSupplier(value, newOldSuppliers);
        }
        
        currentValues = (T[]) new Object[length];
        allocateCachedArrays();
        refresh();
    }

    public void removeSupplier(int startIndex){
        removeSupplierRange(startIndex, 1);
    }

    /**
     * Gets the current value as a double.
     * 
     * @return Double value, or null if not a double type
     */
    public Double getDouble() {
        if (!isDouble || length == 0) {return null;}
        if (signal != null){
            return (double)signal[0].getValueAsDouble();
        }
        return toDouble(currentValues[0]);
    }

    /**
     * Gets the current values as a double array.
     * 
     * @return Array of doubles, or null if not a double type
     */
    public double[] getDoubleArray() {
        if (!isDouble || length == 0) return null;
        
        if (signal != null){
            // Reuse cached array
            if (cachedDoubleArray == null || cachedDoubleArray.length != signal.length) {
                cachedDoubleArray = new double[signal.length];
            }
            for (int i = 0; i < signal.length; i++) {
                cachedDoubleArray[i] = signal[i].getValueAsDouble();
            }
            return cachedDoubleArray;
        }
        return toDoubleArray(currentValues);
    }

    /**
     * Gets the current value as a float (for NT publishing).
     * 
     * @return Float value, or null if not a double type
     */
    public Float getFloat() {
        if (!isDouble || length == 0) {return null;}
        if (signal != null){
            return (float) signal[0].getValueAsDouble();
        }
        Double d = toDouble(currentValues[0]);
        return d != null ? d.floatValue() : null;
    }

    /**
     * Gets the current values as a float array (for NT publishing).
     * 
     * @return Array of floats, or null if not a double type
     */
    public float[] getFloatArray() {
        if (!isDouble || length == 0) return null;
        
        if (signal != null){
            if (cachedFloatArray == null || cachedFloatArray.length != signal.length) {
                cachedFloatArray = new float[signal.length];
            }
            for (int i = 0; i < signal.length; i++) {
                cachedFloatArray[i] = (float) signal[i].getValueAsDouble();
            }
            return cachedFloatArray;
        }
        return toFloatArray(currentValues);
    }

    /**
     * Gets the current value as a boolean.
     * 
     * @return Boolean value, or null if not a boolean type
     */
    public Boolean getBoolean() {
        if (!isBoolean || length == 0) {return null;}
        if (signal != null){
            return (Boolean) signal[0].getValue();
        }
        return (Boolean) currentValues[0];
    }

    /**
     * Gets the current values as a boolean array.
     * 
     * @return Array of booleans, or null if not a boolean type
     */
    public boolean[] getBooleanArray() {
        if (!isBoolean || length == 0) return null;
        
        if (signal != null){
            if (cachedBooleanArray == null || cachedBooleanArray.length != signal.length) {
                cachedBooleanArray = new boolean[signal.length];
            }
            for (int i = 0; i < signal.length; i++) {
                cachedBooleanArray[i] = (Boolean) signal[i].getValue();
            }
            return cachedBooleanArray;
        }
        return toBooleanArray(currentValues);
    }

    /**
     * Gets the current value as a string.
     * 
     * @return String value, or empty string if numeric type
     */
    public String getString() {
        if (isDouble || isBoolean || length == 0) {return "";}
        if (signal != null){
            return signal[0].getValue().toString();
        }
        return currentValues[0] != null ? currentValues[0].toString() : "";
    }

    /**
     * Gets the current values as a string array.
     * 
     * @return Array of strings, or null if numeric type
     */
    public String[] getStringArray() {
        if (isDouble || isBoolean || length == 0) return null;
        
        if (signal != null){
            if (cachedStringArray == null || cachedStringArray.length != signal.length) {
                cachedStringArray = new String[signal.length];
            }
            for (int i = 0; i < signal.length; i++) {
                cachedStringArray[i] = signal[i].getValue().toString();
            }
            return cachedStringArray;
        }
        return toStringArray(currentValues);
    }

    public T getValue() {
        if (length == 0) {return null;}
        return currentValues[0];
    }

    public T[] getValueArray() {
        if (length == 0) {return null;}
        return currentValues;
    }

    public StatusSignal<T> getSignal() {
        return (signal != null && signal.length > 0) ? signal[0] : null;
    }

    public StatusSignal<T>[] getSignals() {
        return signal;
    }

    public Supplier<T> getSupplier() {
        return (oldSupplier != null && oldSupplier.length > 0) ? oldSupplier[0] : null;
    }

    public Supplier<T>[] getSuppliers() {
        return oldSupplier;
    }

    /**
     * Refreshes all values from sources.
     * 
     * <p>For StatusSignals, performs bulk refresh. For Suppliers, calls get().</p>
     */
    public void refresh() {
        changed = false;

        if (signal != null) {
            StatusCode st = StatusSignal.refreshAll(signal);
            if(st == StatusCode.OK) {
                for (int i = 0; i < length; i++){
                    T newVal = signal[i].getValue();
                    if (!Objects.equals(currentValues[i], newVal)) {
                        changed = true;
                    }
                    currentValues[i] = newVal;
                }
            }
        } else {
            for (int i = 0; i < length; i++){
                T newVal = supplier[i].get();

                if (!Objects.equals(currentValues[i], newVal)) {
                    changed = true;
                }
                currentValues[i] = newVal;
            }
        }
    }

    /**
     * Refreshes all Data objects globally.
     * 
     * <p>Efficient bulk refresh of all CTRE signals at once.</p>
     */
    @SuppressWarnings("rawtypes")
    public static void refreshAll() {
        for(Data s : signals) {
            s.refresh();
        }
    }

    /**
     * Checks if the data has changed since last refresh.
     * 
     * <p>Uses configured precision for double comparisons.</p>
     * 
     * @return true if any value changed beyond precision threshold
     */ 
    public boolean hasChanged() {
        return changed;
    }

    /**
     * Gets the timestamp of the data (for CTRE signals).
     * 
     * @return Timestamp in microseconds, or 0 for Suppliers
     */
    public long getTime() {
        if (signal != null) {
            return (long) (signal[0].getTimestamp().getTime() * 1000);
        }
        return 0;
    }

    /**
     * Checks if this Data object contains double values.
     * 
     * @return true if type is numeric
     */
    public boolean isDouble(){
        return isDouble;
    }

    /**
     * Checks if this Data object contains boolean values.
     * 
     * @return true if type is boolean
     */
    public boolean isBoolean(){
        return isBoolean;
    }

    /**
     * Checks if this Data object contains array values.
     * 
     * @return true if multiple values
     */
    public boolean isArray(){
        return isArray;
    }

    private double[] toDoubleArray(T[] value){
        if (value == null) return null;

        if (isArray){
            int arrayLength = java.lang.reflect.Array.getLength(value[0]);
            if (cachedDoubleArray == null || cachedDoubleArray.length != arrayLength) {
                cachedDoubleArray = new double[arrayLength];
            }
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                cachedDoubleArray[i] = (elem != null) ? ((Number) elem).doubleValue() : 0.0;
            }
            return cachedDoubleArray;
        } else {
            if (cachedDoubleArray == null || cachedDoubleArray.length != length) {
                cachedDoubleArray = new double[length];
            }
            for (int i = 0; i < length; i++) {
                cachedDoubleArray[i] = (value[i] != null) ? ((Number) value[i]).doubleValue() : 0.0;
            }
            return cachedDoubleArray;
        }
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

        if (isArray){
            int arrayLength = java.lang.reflect.Array.getLength(value[0]);
            if (cachedFloatArray == null || cachedFloatArray.length != arrayLength) {
                cachedFloatArray = new float[arrayLength];
            }
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                cachedFloatArray[i] = (elem != null) ? ((Number) elem).floatValue() : 0f;
            }
            return cachedFloatArray;
        } else {
            if (cachedFloatArray == null || cachedFloatArray.length != length) {
                cachedFloatArray = new float[length];
            }
            for (int i = 0; i < length; i++) {
                cachedFloatArray[i] = (value[i] != null) ? ((Number) value[i]).floatValue() : 0f;
            }
            return cachedFloatArray;
        }
    }

    private boolean[] toBooleanArray(T[] value){
        if (value == null) return null;

        if (isArray){
            int arrayLength = java.lang.reflect.Array.getLength(value[0]);
            if (cachedBooleanArray == null || cachedBooleanArray.length != arrayLength) {
                cachedBooleanArray = new boolean[arrayLength];
            }
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                cachedBooleanArray[i] = (elem != null) && (Boolean) elem;
            }
            return cachedBooleanArray;
        } else {
            if (cachedBooleanArray == null || cachedBooleanArray.length != length) {
                cachedBooleanArray = new boolean[length];
            }
            for (int i = 0; i < length; i++) {
                cachedBooleanArray[i] = (value[i] != null) && (Boolean) value[i];
            }
            return cachedBooleanArray;
        }
    }
    
    private String[] toStringArray(T[] value){
        if (value == null) return null;

        if (isArray){
            int arrayLength = java.lang.reflect.Array.getLength(value[0]);
            if (cachedStringArray == null || cachedStringArray.length != arrayLength) {
                cachedStringArray = new String[arrayLength];
            }
            for (int i = 0; i < arrayLength; i++) {
                Object elem = java.lang.reflect.Array.get(value[0], i);
                cachedStringArray[i] = (elem != null) ? elem.toString() : null;
            }
            return cachedStringArray;
        } else {
            if (cachedStringArray == null || cachedStringArray.length != length) {
                cachedStringArray = new String[length];
            }
            for (int i = 0; i < length; i++) {
                cachedStringArray[i] = (value[i] != null) ? value[i].toString() : null;
            }
            return cachedStringArray;
        }
    }

    public void cleanup() {
        signals.remove(this);
        cachedDoubleArray = null;
        cachedFloatArray = null;
        cachedBooleanArray = null;
        cachedStringArray = null;
    }
    
    public static void clearAllSignals() {
        signals.clear();
    }
}