package frc.demacia.SysID;

import java.io.DataInputStream;
import java.io.EOFException;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.simple.SimpleMatrix;

public class LogReader {

    private static Map<Integer,List<EntryDescription>> entries = new HashMap<>();
    private static double minPowerToMove = Double.MAX_VALUE;

    public enum MechanismType {
        POSITION,
        VELOCITY,
        ELEVATOR,
        ARM
    }

    private static class EntryDescription {
        String name;
        String type;
        List<DataPoint> data = new ArrayList<>();

        EntryDescription(String name, String type) {
            this.name = name;
            this.type = type;
        }
    }

    private static class DataPoint {
        long timestamp;
        double[] value;

        DataPoint(long timestamp, double[] value) {
            this.timestamp = timestamp;
            this.value = value.clone();
        }
    }

    public static class SysIDResults {
        public BucketResult finalFit;

        public SysIDResults(BucketResult finalFit) {
            this.finalFit = finalFit;
        }

        @Override
        public String toString() {
            return finalFit != null ? finalFit.toString() : "No Result";
        }
    }

    public static void loadFile(String fileName) {
        entries = new HashMap<>();
        try {
            System.out.println("Reading log file: " + fileName);
            wpilogReader(fileName);
        } catch (IOException e) {
            System.err.println("Error reading log file: " + e.getMessage());
            e.printStackTrace();
        }
    }

    public static Map<String, SysIDResults> analyze(MechanismType type) {
        if (entries == null || entries.isEmpty()) return new HashMap<>();
        System.out.println("Analyzing as " + type);
        return performAnalysis(type);
    }

    public static Map<String, SysIDResults> getResult(String fileName) {
        loadFile(fileName);
        return analyze(MechanismType.POSITION);
    }

    private static void wpilogReader(String fileName) throws IOException {
        try (FileInputStream fileInputStream = new FileInputStream(fileName);
             DataInputStream dataInputStream = new DataInputStream(fileInputStream)) {
            
            byte[] signature = readHeader(dataInputStream);
            if (!Arrays.equals(signature, "WPILOG".getBytes())) {
                throw new IOException("Invalid WPILOG file format. Expected WPILOG, got: " + new String(signature));
            }
            
            skipHeaderExtra(dataInputStream);
            readRecords(dataInputStream);
        }
    }

    private static byte[] readHeader(DataInputStream dataInputStream) throws IOException {
        byte[] signature = new byte[6];
        dataInputStream.readFully(signature);
        return signature;
    }

    private static void skipHeaderExtra(DataInputStream dataInputStream) throws IOException {
        short version = Short.reverseBytes(dataInputStream.readShort());
        int extraLength = Integer.reverseBytes(dataInputStream.readInt());
        System.out.println("WPILOG version: " + version + ", extra header length: " + extraLength);
        if(extraLength > 0) {
            dataInputStream.skipBytes(extraLength);
        }
    }

    private static void readRecords(DataInputStream dataInputStream) throws IOException {
        int recordCount = 0;
        int dataRecordsProcessed = 0;
        
        while (true) {
            try {
                if (readRecord(dataInputStream)) {
                    dataRecordsProcessed++;
                }
                recordCount++;
            } catch (EOFException e) {
                break;
            }
        }
        System.out.println("Total records scanned: " + recordCount);
        System.out.println("Valid data records stored: " + dataRecordsProcessed);
    }

    private static boolean readRecord(DataInputStream dataInputStream) throws IOException {
        int headerByte = dataInputStream.readUnsignedByte();
        int idLength = (headerByte & 0x3) + 1;
        int payloadLength = (headerByte >> 2 & 0x3) + 1;
        int timestampLength = (headerByte >> 4 & 0x7) + 1;

        int recordId = readLittleEndianInt(dataInputStream, idLength);
        int payloadSize = readLittleEndianInt(dataInputStream, payloadLength);
        long timestamp = readLittleEndianLong(dataInputStream, timestampLength);

        if (recordId == 0) {
            addEntryFromControlRecord(dataInputStream, payloadSize);
            return false;
        } else {
            List<EntryDescription> entryList = entries.get(recordId);
            if (entryList != null && !entryList.isEmpty()) {
                String type = entryList.get(0).type.trim();

                boolean isFloat = type.equals("float") || type.equals("float[]");
                boolean isDouble = type.equals("double") || type.equals("double[]");

                if (isFloat || isDouble) {
                    double[] value = null;

                    if (isFloat) {
                        if (payloadSize % 4 == 0) {
                            int count = payloadSize / 4;
                            value = new double[count];
                            for (int i = 0; i < count; i++) {
                                int raw = Integer.reverseBytes(dataInputStream.readInt());
                                value[i] = (double) Float.intBitsToFloat(raw);
                            }
                        } else {
                            dataInputStream.skipBytes(payloadSize);
                        }
                    } else if (isDouble) {
                         if (payloadSize % 8 == 0) {
                            int count = payloadSize / 8;
                            value = new double[count];
                            for (int i = 0; i < count; i++) {
                                long raw = Long.reverseBytes(dataInputStream.readLong());
                                value[i] = Double.longBitsToDouble(raw);
                            }
                        } else {
                            dataInputStream.skipBytes(payloadSize);
                        }
                    }

                    if (value != null) {
                        int numEntries = entryList.size();
                        if (numEntries > 1 && value.length % numEntries == 0) {
                            int chunkSize = value.length / numEntries;
                            for (int i = 0; i < numEntries; i++) {
                                double[] slice = Arrays.copyOfRange(value, i * chunkSize, (i + 1) * chunkSize);
                                entryList.get(i).data.add(new DataPoint(timestamp, slice));
                            }
                        } else {
                            for (EntryDescription entry : entryList) {
                                entry.data.add(new DataPoint(timestamp, value));
                            }
                        }
                        return true;
                    }

                } else {
                    dataInputStream.skipBytes(payloadSize);
                }
            } else {
                dataInputStream.skipBytes(payloadSize);
            }
        }
        return false;
    }

    private static void addEntryFromControlRecord(DataInputStream dataInputStream, int payloadSize) throws IOException {
        int recordType = dataInputStream.readUnsignedByte();
        if (recordType == 0) {
            int entryId = Integer.reverseBytes(dataInputStream.readInt());
            int nameLength = Integer.reverseBytes(dataInputStream.readInt());
            String name = readString(dataInputStream, nameLength);
            int typeLength = Integer.reverseBytes(dataInputStream.readInt());
            String type = readString(dataInputStream, typeLength);
            int metaLength = Integer.reverseBytes(dataInputStream.readInt());
            String metadata = readString(dataInputStream, metaLength);

            String[] names = name.split(" \\| ");
            String[] metas = metadata.split(" \\| ");
            
            for (int i = 0; i < names.length; i++) {
                String currentName = names[i].trim();
                String currentMeta = (i < metas.length) ? metas[i].trim() : "";

                if (currentMeta.contains("motor")) {
                    entries.putIfAbsent(entryId, new ArrayList<>());
                    entries.get(entryId).add(new EntryDescription(currentName, type));
                }
            }
        } else {
            if (payloadSize > 1) {
                dataInputStream.skipBytes(payloadSize - 1);
            }
        }
    }

    private static String readString(DataInputStream dataInputStream, int length) throws IOException {
        byte[] bytes = new byte[length];
        dataInputStream.readFully(bytes);
        return new String(bytes, "UTF-8");
    }

    private static int readLittleEndianInt(DataInputStream dis, int bytes) throws IOException {
        int result = 0;
        for (int i = 0; i < bytes; i++) {
            result |= (dis.readUnsignedByte() << (i * 8));
        }
        return result;
    }

    private static long readLittleEndianLong(DataInputStream dis, int bytes) throws IOException {
        long result = 0L;
        for (int i = 0; i < bytes; i++) {
            result |= ((long) dis.readUnsignedByte() << (i * 8));
        }
        return result;
    }

    private static Map<String, SysIDResults> performAnalysis(MechanismType type) {
        Map<String, SysIDResults> results = new HashMap<>();
        Set<String> groups = findGroups();
        
        for (String group : groups) {
            SysIDResults result = analyzeGroup(group, type);
            if (result != null) {
                results.put(group, result);
            }
        }
        return results;
    }

    private static Set<String> findGroups() {
        Set<String> groups = new HashSet<>();
        for (List<EntryDescription> list : entries.values()) {
            for (EntryDescription entry : list) {
                groups.add(entry.name);
            }
        }
        return groups;
    }

    private static SysIDResults analyzeGroup(String name, MechanismType type) {
        List<DataPoint> allData = new ArrayList<>();

        for (List<EntryDescription> list : entries.values()) {
            for (EntryDescription entry : list) {
                if (entry.name.equals(name)) {
                    allData.addAll(entry.data);
                }
            }
        }

        if (allData.isEmpty()) {
            return null;
        }

        allData.sort((p1, p2) -> Long.compare(p1.timestamp, p2.timestamp));

        List<SyncedDataPoint> syncedData = synchronizeData(allData);
        return performSysIdLikeAnalysis(syncedData, type);
    }

    public static class SyncedDataPoint {
        double velocity, position, acceleration, rawAcceleration, voltage;
        long timestamp;
        SyncedDataPoint prev;

        SyncedDataPoint(double velocity, double position, double acceleration, double voltage, long timestamp) {
            this.velocity = velocity;
            this.position = position;
            this.acceleration = acceleration;
            this.voltage = voltage;
            this.timestamp = timestamp;
        }
    }

    private static List<SyncedDataPoint> synchronizeData(List<DataPoint> dataPoints) {
        List<SyncedDataPoint> result = new ArrayList<>();
        for (DataPoint dp : dataPoints) {
            if (dp.value.length >= 4) {
                result.add(new SyncedDataPoint(dp.value[1], dp.value[0], dp.value[2], dp.value[3], dp.timestamp));
            }
        }
        updateAccelerationAndMinPower(result);
        return result;
    }

    private static void updateAccelerationAndMinPower(List<SyncedDataPoint> data) {
        minPowerToMove = Double.MAX_VALUE;
        
        int windowSize = 5; 

        for (int i = 0; i < data.size(); i++) {
            SyncedDataPoint current = data.get(i);
            
            if (i >= windowSize && i < data.size() - windowSize) {
                SyncedDataPoint future = data.get(i + windowSize);
                SyncedDataPoint past = data.get(i - windowSize);
                
                double dt = (future.timestamp - past.timestamp) / 1_000_000.0;
                if (dt > 0) {
                    double smoothAcc = (future.velocity - past.velocity) / dt;
                    current.acceleration = (current.acceleration + smoothAcc) / 2.0; 
                }
            } else if (current.prev != null) {
                double dt = (current.timestamp - current.prev.timestamp) / 1_000_000.0;
                if (dt <= 0) dt = 1e-6;
                double rawAcc = (current.velocity - current.prev.velocity) / dt;
                current.acceleration = (current.acceleration + rawAcc) / 2.0;
            }

            if (current.prev != null) {
                 double absVolt = Math.abs(current.voltage);
                 if (current.prev.velocity == 0 && current.velocity != 0 && absVolt > 0.01 && 
                    (current.velocity * current.voltage) > 0 && absVolt < minPowerToMove) {
                    minPowerToMove = absVolt;
                }
            }
            current.prev = (i > 0) ? data.get(i-1) : null;
        }
        if (minPowerToMove == Double.MAX_VALUE) minPowerToMove = 0.0;
    }

    public static class BucketResult {
        double ks, kv, ka, kg, kp, avgError, maxError;
        int points;

        BucketResult(double ks, double kv, double ka, double kg, double kp, double avgError, double maxError, int points) {
            this.ks = ks;
            this.kv = kv;
            this.ka = ka;
            this.kg = kg;
            this.kp = kp;
            this.avgError = avgError;
            this.maxError = maxError;
            this.points = points;
        }

        @Override
        public String toString() {
            return String.format("KS=%.4f, KV=%.4f, KA=%.4f, KG=%.4f, KP=%.4f, AvgError=%.2f%%, MaxError=%.2f%%, Points=%d", 
                               ks, kv, ka, kg, kp, avgError*100, maxError*100, points);
        }
    }

    private static SysIDResults performSysIdLikeAnalysis(List<SyncedDataPoint> data, MechanismType type) {
        if (data.isEmpty()) return new SysIDResults(null);

        List<SyncedDataPoint> validData = new ArrayList<>();
        double maxV = 0.0;
        for (SyncedDataPoint d : data) {
            maxV = Math.max(maxV, Math.abs(d.velocity));
            if (Math.abs(d.velocity) > 0.01 || Math.abs(d.voltage) > 0.01) {
                validData.add(d);
            }
        }

        List<SyncedDataPoint> slowData = new ArrayList<>();
        List<SyncedDataPoint> fastData = new ArrayList<>();
        
        double slowThresh = maxV * 0.3;
        double fastThresh = maxV * 0.7;

        for (SyncedDataPoint d : validData) {
            double vAbs = Math.abs(d.velocity);
            if (vAbs < slowThresh) {
                slowData.add(d);
            }
            if (vAbs > fastThresh) {
                fastData.add(d);
            }
        }

        BucketResult ksResult = solveBucket(slowData, type, -1, null);
        double foundKs = (ksResult != null) ? ksResult.ks : 0.0;

        BucketResult allDataResult = solveBucket(validData, type, foundKs, fastData);

        return new SysIDResults(allDataResult);
    }

    private static double measureObservedKvRANSAC(List<SyncedDataPoint> data, double knownKs, double knownKg, MechanismType type) {
        if (data.isEmpty()) return 0;

        List<SyncedDataPoint> steadyStateCandidates = new ArrayList<>();
        for (SyncedDataPoint p : data) {
            if (Math.abs(p.velocity) > 0.1 && Math.abs(p.acceleration) < 1.0) {
                steadyStateCandidates.add(p);
            }
        }

        if (steadyStateCandidates.size() < 10) return 0;

        int iterations = 100;
        double bestKv = 0;
        int maxInliers = -1;
        double voltageThreshold = 0.25;

        for (int i = 0; i < iterations; i++) {
            SyncedDataPoint randomPoint = steadyStateCandidates.get((int) (Math.random() * steadyStateCandidates.size()));
            
            double sign = Math.signum(randomPoint.velocity);
            double friction = knownKs * sign;
            double gravity = 0.0;

            if (type == MechanismType.ELEVATOR) {
                gravity = knownKg;
            } else if (type == MechanismType.ARM) {
                gravity = knownKg * Math.cos(randomPoint.position);
            }

            double netVoltage = randomPoint.voltage - friction - gravity;
            
            if (Math.signum(netVoltage) != sign) continue;

            double candidateKv = netVoltage / randomPoint.velocity;
            int inliers = 0;

            for (SyncedDataPoint p : steadyStateCandidates) {
                double pSign = Math.signum(p.velocity);
                double pFriction = knownKs * pSign;
                double pGravity = 0.0;

                if (type == MechanismType.ELEVATOR) {
                    pGravity = knownKg;
                } else if (type == MechanismType.ARM) {
                    pGravity = knownKg * Math.cos(p.position);
                }

                double expectedVoltage = (candidateKv * p.velocity) + pFriction + pGravity;
                if (Math.abs(p.voltage - expectedVoltage) < voltageThreshold) {
                    inliers++;
                }
            }

            if (inliers > maxInliers) {
                maxInliers = inliers;
                bestKv = candidateKv;
            }
        }

        double sumKv = 0;
        int count = 0;
        
        for (SyncedDataPoint p : steadyStateCandidates) {
            double pSign = Math.signum(p.velocity);
            double pFriction = knownKs * pSign;
            double pGravity = 0.0;

            if (type == MechanismType.ELEVATOR) {
                pGravity = knownKg;
            } else if (type == MechanismType.ARM) {
                pGravity = knownKg * Math.cos(p.position);
            }

            double expectedVoltage = (bestKv * p.velocity) + pFriction + pGravity;
            if (Math.abs(p.voltage - expectedVoltage) < voltageThreshold) {
                double netV = p.voltage - pFriction - pGravity;
                if(Math.abs(p.velocity) > 0.001) {
                    sumKv += (netV / p.velocity);
                    count++;
                }
            }
        }

        return count > 0 ? sumKv / count : bestKv;
    }

    private static BucketResult solveBucket(List<SyncedDataPoint> sourceData, MechanismType type, double fixedKs, List<SyncedDataPoint> fastDataForCheck) {
        if (sourceData == null || sourceData.isEmpty()) return null;
        if (sourceData.size() < 10) return null;

        int rows = sourceData.size();
        boolean useFixedKs = (fixedKs >= 0);
        int baseCols = (type == MechanismType.VELOCITY || type == MechanismType.POSITION) ? 3 : 4;
        int cols = useFixedKs ? (baseCols - 1) : baseCols;

        SimpleMatrix mat = new SimpleMatrix(rows, cols);
        SimpleMatrix volt = new SimpleMatrix(rows, 1);
        
        for (int r = 0; r < rows; r++) {
            SyncedDataPoint d = sourceData.get(r);
            int colIdx = 0;
            
            if (!useFixedKs) {
                mat.set(r, colIdx++, Math.signum(d.velocity));
            }
            mat.set(r, colIdx++, d.velocity);
            mat.set(r, colIdx++, d.acceleration);
            
            if (baseCols == 4) {
                if (type == MechanismType.ELEVATOR) {
                    mat.set(r, colIdx++, 1.0);
                } else if (type == MechanismType.ARM) {
                    mat.set(r, colIdx++, Math.cos(d.position));
                }
            }
            
            double effectiveVoltage = d.voltage;
            if (useFixedKs) {
                effectiveVoltage -= (fixedKs * Math.signum(d.velocity));
            }
            volt.set(r, 0, effectiveVoltage);
        }
        
        try {
            SimpleMatrix res = mat.solve(volt);
            
            double calculatedKs, calculatedKv, calculatedKa, calculatedKg = 0;
            int resIdx = 0;
            
            if (useFixedKs) {
                calculatedKs = fixedKs;
            } else {
                calculatedKs = res.get(resIdx++, 0);
            }
            
            calculatedKv = res.get(resIdx++, 0);
            calculatedKa = res.get(resIdx++, 0);
            
            if (baseCols == 4) {
                calculatedKg = res.get(resIdx++, 0);
            }
            
            if (useFixedKs && fastDataForCheck != null && !fastDataForCheck.isEmpty()) {
                double currentKg = (baseCols == 4) ? calculatedKg : 0.0;
                
                double observedKv = measureObservedKvRANSAC(fastDataForCheck, calculatedKs, currentKg, type);
                
                if (observedKv > 0) {
                    boolean olsIsBad = calculatedKv <= 0;
                    boolean olsIsOutlier = Math.abs(calculatedKv - observedKv) > (observedKv * 0.5);

                    if (olsIsBad || olsIsOutlier) {
                        System.out.println("FIX: Math Kv (" + calculatedKv + ") replaced by Observed Kv (" + observedKv + ")");
                        calculatedKv = observedKv;
                    } else {
                        System.out.println("Check Passed: Math Kv (" + calculatedKv + ") matches Observed Kv (" + observedKv + ")");
                    }
                }
            }

            double sumError = 0;
            double maxError = 0;
            int validCount = 0;
            
            for (SyncedDataPoint d : sourceData) {
                if (Math.abs(d.voltage) > 0.001) {
                    double predVolts = (calculatedKs * Math.signum(d.velocity)) + (calculatedKv * d.velocity) + (calculatedKa * d.acceleration);
                    if (baseCols == 4) {
                        double gravFactor = (type == MechanismType.ELEVATOR) ? 1.0 : Math.cos(d.position);
                        predVolts += (calculatedKg * gravFactor);
                    }
                    
                    double relError = Math.abs(d.voltage - predVolts) / Math.abs(d.voltage);
                    sumError += relError;
                    maxError = Math.max(maxError, relError);
                    validCount++;
                }
            }
            double avgError = validCount > 0 ? sumError / validCount : 0;
            double kp = CalculateFeedbackGains.calculateFeedbackGains(calculatedKv, calculatedKa, type);
            
            return new BucketResult(calculatedKs, calculatedKv, calculatedKa, calculatedKg, kp, avgError, maxError, rows);
        } catch (Exception e) {
            System.err.println("Error solving bucket: " + e.getMessage());
            return null;
        }
    }

    public static class CalculateFeedbackGains {
        private static final double MIN_PHYSICAL_KA = 0.05;

        public static double calculateFeedbackGains(double kv, double ka, MechanismType type) {
            double absKa = Math.abs(ka);
            double effectiveKa = Math.max(absKa, MIN_PHYSICAL_KA);

            if (type == MechanismType.VELOCITY) {
                return (2.0 * kv) / effectiveKa;
            } else {
                return (kv * kv) / (4.0 * effectiveKa);
            }
        }
    }
}