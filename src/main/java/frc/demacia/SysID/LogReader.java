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

    private static Map<Integer, EntryDescription> entries;
    private static double minPowerToMove = Double.MAX_VALUE;

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
        public BucketResult slow;
        public BucketResult mid;
        public BucketResult high;

        public SysIDResults(BucketResult slow, BucketResult mid, BucketResult high) {
            this.slow = slow;
            this.mid = mid;
            this.high = high;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append("Slow: ").append(slow != null ? slow.toString() : "null").append("\n");
            sb.append("Mid: ").append(mid != null ? mid.toString() : "null").append("\n");
            sb.append("High: ").append(high != null ? high.toString() : "null");
            return sb.toString();
        }
    }

    public static Map<String, SysIDResults> getResult(String fileName) {
        entries = new HashMap<>();
        try {
            System.out.println("Reading log file: " + fileName);
            wpilogReader(fileName);
            System.out.println("Found " + entries.size() + " motor entries");
            return performAnalysis();
        } catch (IOException e) {
            System.err.println("Error reading log file: " + e.getMessage());
            e.printStackTrace();
            return new HashMap<>();
        }
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
        dataInputStream.skipBytes(extraLength);
    }

    private static void readRecords(DataInputStream dataInputStream) throws IOException {
        int recordCount = 0;
        while (dataInputStream.available() > 0) {
            try {
                readRecord(dataInputStream);
                recordCount++;
                if (recordCount % 1000 == 0) {
                    System.out.println("Processed " + recordCount + " records");
                }
            } catch (EOFException e) {
                break;
            }
        }
        System.out.println("Total records processed: " + recordCount);
    }

    private static void readRecord(DataInputStream dataInputStream) throws IOException {
        int headerByte = dataInputStream.readUnsignedByte();
        int idLength = (headerByte & 0x3) + 1;
        int payloadLength = (headerByte >> 2 & 0x3) + 1;
        int timestampLength = (headerByte >> 4 & 0x7) + 1;

        int recordId = readLittleEndianInt(dataInputStream, idLength);
        int payloadSize = readLittleEndianInt(dataInputStream, payloadLength);
        long timestamp = readLittleEndianLong(dataInputStream, timestampLength);

        if (recordId == 0) {
            addEntryFromControlRecord(dataInputStream, payloadSize);
        } else {
            EntryDescription entry = entries.get(recordId);
            if (entry != null && "float[]".equals(entry.type)) {
                if (payloadSize % 4 == 0) {
                    int count = payloadSize / 4;
                    double[] value = new double[count];
                    for (int i = 0; i < count; i++) {
                        int raw = Integer.reverseBytes(dataInputStream.readInt());
                        value[i] = (double) Float.intBitsToFloat(raw);
                    }
                    entry.data.add(new DataPoint(timestamp, value));
                } else {
                    System.err.println("Invalid payload size for float[] entry: " + entry.name);
                    dataInputStream.skipBytes(payloadSize);
                }
            } else {
                dataInputStream.skipBytes(payloadSize);
            }
        }
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

            System.out.println("Found entry: " + name + " (type: " + type + ", metadata: " + metadata + ")");

            if ("motor".equals(metadata)) {
                entries.put(entryId, new EntryDescription(name, type));
                System.out.println("Added motor entry: " + name);
            }
        } else {
            dataInputStream.skipBytes(payloadSize - 1);
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

    private static Map<String, SysIDResults> performAnalysis() {
        Map<String, SysIDResults> results = new HashMap<>();
        Set<String> groups = findGroups();
        System.out.println("Found " + groups.size() + " motor groups");
        
        for (String group : groups) {
            System.out.println("Analyzing group: " + group);
            SysIDResults result = analyzeGroup(group);
            if (result != null) {
                results.put(group, result);
                System.out.println("Group " + group + ": " + result);
            } else {
                System.out.println("No valid data for group: " + group);
            }
        }
        return results;
    }

    private static Set<String> findGroups() {
        Set<String> groups = new HashSet<>();
        for (EntryDescription entry : entries.values()) {
            String name = entry.name;
            groups.add(name);
            System.out.println("Added group: " + name);
        }
        return groups;
    }

    private static SysIDResults analyzeGroup(String name) {
        EntryDescription entry = findEntry(name);
        if (entry != null) {
            System.out.println("Found entry with name: " + name);
        }

        if (entry == null || entry.data.isEmpty()) {
            System.out.println("No entry found for group: " + name);
            return null;
        }

        System.out.println("Entry has " + entry.data.size() + " data points");
        List<SyncedDataPoint> syncedData = synchronizeData(entry.data);
        System.out.println("Synchronized data has " + syncedData.size() + " points");

        return performSysIdLikeAnalysis(syncedData);
    }

    private static EntryDescription findEntry(String name) {
        for (EntryDescription entry : entries.values()) {
            if (entry.name.equals(name)) return entry;
        }
        return null;
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
        SyncedDataPoint prev = null;
        for (SyncedDataPoint m : data) {
            m.rawAcceleration = m.acceleration;
            if (prev != null) {
                double deltaTime = (m.timestamp - prev.timestamp) / 1000000.0;
                if (deltaTime <= 0) deltaTime = 1e-6;
                double acc = (m.velocity - prev.velocity) / deltaTime;
                m.acceleration = (m.acceleration * deltaTime + acc * 0.02) / (deltaTime + 0.02);

                double absVolt = Math.abs(m.voltage);
                if (prev.velocity == 0 && m.velocity != 0 && absVolt > 0.01 && 
                    (m.velocity * m.voltage) > 0 && absVolt < minPowerToMove) {
                    minPowerToMove = absVolt;
                }
            }
            m.prev = prev;
            prev = m;
        }
        if (minPowerToMove == Double.MAX_VALUE) minPowerToMove = 0.0;
    }

    public static class BucketResult {
        double ks, kv, ka, kp, avgError, maxError;
        int points;

        BucketResult(double ks, double kv, double ka, double kp, double avgError, double maxError, int points) {
            this.ks = ks;
            this.kv = kv;
            this.ka = ka;
            this.kp = kp;
            this.avgError = avgError;
            this.maxError = maxError;
            this.points = points;
        }

        @Override
        public String toString() {
            return String.format("KS=%.4f, KV=%.4f, KA=%.4f, KP=%.4f, AvgError=%.2f%%, MaxError=%.2f%%, Points=%d", 
                               ks, kv, ka, kp, avgError*100, maxError*100, points);
        }
    }

    private static SysIDResults performSysIdLikeAnalysis(List<SyncedDataPoint> data) {
        if (data.isEmpty()) return new SysIDResults(null, null, null);

        double maxV = 0.0;
        for (SyncedDataPoint m : data) {
            maxV = Math.max(maxV, Math.abs(m.velocity));
        }

        double[] vRange = new double[]{maxV * 0.3, maxV * 0.7, maxV};
        List<SyncedDataPoint> slow = new ArrayList<>();
        List<SyncedDataPoint> mid = new ArrayList<>();
        List<SyncedDataPoint> high = new ArrayList<>();

        for (SyncedDataPoint d : data) {
            int r = rangeBucket(d, vRange);
            if (r == 0) slow.add(d);
            else if (r == 1) mid.add(d);
            else if (r == 2) high.add(d);
        }

        System.out.println("Data distribution - Slow: " + slow.size() + ", Mid: " + mid.size() + ", High: " + high.size());

        BucketResult slowR = solveBucket(slow);
        BucketResult midR = solveBucket(mid);
        BucketResult highR = solveBucket(high);

        return new SysIDResults(slowR, midR, highR);
    }

    private static int rangeBucket(SyncedDataPoint d, double[] vRange) {
        double vAbs = Math.abs(d.velocity);
        int i = vAbs < vRange[0] ? 0 : (vAbs < vRange[1] ? 1 : 2);
        
        if (valid(vAbs, 0.1) && valid(d.voltage, 0.05)) {
            if (d.prev != null) {
                if (valid(Math.abs(d.prev.velocity), 0.1) && valid(Math.abs(d.prev.voltage), 0.2)) {
                    return i;
                } else {
                    return -1;
                }
            } else {
                return i;
            }
        } else {
            return -1;
        }
    }

    private static boolean valid(double val, double min) {
        return Math.abs(val) > min;
    }

    private static BucketResult solveBucket(List<SyncedDataPoint> arr) {
        if (arr == null || arr.size() <= 50) return null;
        
        int rows = arr.size();
        SimpleMatrix mat = new SimpleMatrix(rows, 3);
        SimpleMatrix volt = new SimpleMatrix(rows, 1);
        
        for (int r = 0; r < rows; r++) {
            SyncedDataPoint d = arr.get(r);
            mat.set(r, 0, Math.signum(d.velocity));
            mat.set(r, 1, d.velocity);
            mat.set(r, 2, d.acceleration);
            volt.set(r, 0, d.voltage);
        }
        
        try {
            SimpleMatrix res = mat.solve(volt);
            SimpleMatrix pred = mat.mult(res);
            SimpleMatrix error = volt.minus(pred);
            
            double maxError = 0;
            double sumError = 0;
            int validCount = 0;
            
            for (int i = 0; i < rows; i++) {
                SyncedDataPoint d = arr.get(i);
                if (Math.abs(d.voltage) > 0.001) {
                    double relError = Math.abs(error.get(i, 0) / d.voltage);
                    sumError += relError;
                    maxError = Math.max(maxError, relError);
                    validCount++;
                }
            }
            double avgError = validCount > 0 ? sumError / validCount : 0;
            
            double kp = CalculateFeedbackGains.calculateFeedbackGains(res.get(1, 0), res.get(2, 0));
            
            return new BucketResult(minPowerToMove, res.get(1, 0), res.get(2, 0), kp, avgError, maxError, rows);
        } catch (Exception e) {
            System.err.println("Error solving bucket: " + e.getMessage());
            return null;
        }
    }

    public static class CalculateFeedbackGains {
        public static double calculateFeedbackGains(double kv, double ka) {
            if (Math.abs(ka) < 1e-10) return 0.0;
            double kP = (2.0 * kv) / ka;
            return kP;
        }
    }
}