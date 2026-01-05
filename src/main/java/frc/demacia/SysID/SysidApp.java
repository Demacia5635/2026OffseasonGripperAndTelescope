package frc.demacia.SysID;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;
import java.util.List;
import java.util.function.Consumer;

public class SysidApp {
    public static void main(String[] args) {
        Sysid app = new Sysid();
        app.show();
    }
}

class Sysid implements Consumer<File> {
    JFrame frame = new JFrame("Sysid");
    FileChooserPanel fileChooser = new FileChooserPanel(this);
    DefaultListModel<MotorData> listModel = new DefaultListModel<>();
    JList<MotorData> motorList = new JList<>(listModel);
    SysidResultPanel result = new SysidResultPanel(this);
    JTextArea msgArea = new JTextArea();
    JScrollPane msgPane = new JScrollPane(msgArea, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_NEVER);
    
    File currentFile;

    private static Sysid sysid = null;

    public Sysid() {
        sysid = this;
        frame.setSize(1024,800);
        frame.setMinimumSize(new Dimension(1024,800));
        frame.setLocation(300,200);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        var pane = frame.getContentPane();
        pane.setLayout(new GridBagLayout());
        motorList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        motorList.setMinimumSize(new Dimension(300,400));
        motorList.setBorder(BorderFactory.createEtchedBorder());
        
        motorList.addListSelectionListener(e -> {
            if (!e.getValueIsAdjusting()) {
                MotorData selected = motorList.getSelectedValue();
                if (selected != null) {
                    msg("Selected motor: " + selected.name);
                    result.updateDisplay(selected);
                }
            }
        });
        
        pane.add(fileChooser, new GridBagConstraints(0, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.HORIZONTAL, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(motorList, new GridBagConstraints(0, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(msgPane, new GridBagConstraints(1, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        pane.add(result, new GridBagConstraints(1, 1, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(5, 5, 5, 0), 5, 5));
        frame.pack();
    }

    public void show() {
        frame.setVisible(true);
    }

    public static void msg(String msg) {
        if(sysid != null) {
            sysid.msgArea.append(msg + "\n");
            sysid.msgArea.setCaretPosition(sysid.msgArea.getDocument().getLength());
        }
    }

    @Override
    public void accept(File file) {
        System.out.println("File set to " + file);
        this.currentFile = file;
        LogReader.loadFile(file.getAbsolutePath());
        analyzeFile(LogReader.MechanismType.POSITION);
    }

    public void analyzeFile(LogReader.MechanismType type) {
        if (currentFile == null) return;
        
        MotorData currentlySelected = motorList.getSelectedValue();
        String selectedName = (currentlySelected != null) ? currentlySelected.name : null;

        try {
            MotorData.motors.clear();
            listModel.clear();
            
            Map<String, LogReader.SysIDResults> analysisResults = LogReader.analyze(type);
            
            msg("Analysis complete (" + type + "). Found " + analysisResults.size() + " groups.");
            
            int newIndexToSelect = 0;
            int counter = 0;

            List<String> sortedNames = new ArrayList<>(analysisResults.keySet());
            Collections.sort(sortedNames);

            for(String groupName : sortedNames) {
                MotorData motorData = new MotorData();
                motorData.name = groupName;
                motorData.sysidResult = analysisResults.get(groupName);
                MotorData.motors.add(motorData);
                listModel.addElement(motorData);

                if (selectedName != null && selectedName.equals(groupName)) {
                    newIndexToSelect = counter;
                }
                counter++;
            }
            
            if (!listModel.isEmpty()) {
                motorList.setSelectedIndex(newIndexToSelect);
                result.updateDisplay(motorList.getSelectedValue());
            }
            
        } catch (Exception e) {
            msg("Analysis error: " + e);
            e.printStackTrace();
        }
    }

    public MotorData getMotor() {
        return motorList.getSelectedValue();
    }
}

class FileChooserPanel extends JPanel implements ActionListener {
    JButton button;
    JTextField field;
    JFileChooser chooser;
    Consumer<File> consumer;

    public FileChooserPanel(Consumer<File> consumer) {
        super(new FlowLayout(FlowLayout.LEFT, 10, 5));
        this.consumer = consumer;
        button = new JButton("File:");
        field = new JTextField(50);
        field.setEditable(false);
        button.addActionListener(this);
        chooser = new JFileChooser(System.getProperty("user.dir"));
        chooser.setFileFilter(new FileNameExtensionFilter("wpi log", "wpilog"));
        add(button);
        add(field);
    }

    @Override
    public void actionPerformed(ActionEvent e) {
        int res = chooser.showOpenDialog(null);
        if(res == JFileChooser.APPROVE_OPTION) {
            File file = chooser.getSelectedFile();
            field.setText(file.getAbsolutePath());
            if(consumer != null) {
                consumer.accept(file);
            }
        }
    }
}

class SysidResultPanel extends JPanel {
    public static int nK = KTypes.values().length;
    JCheckBox[] checkBoxes;
    JLabel[][] k;
    
    JComboBox<LogReader.MechanismType> mechTypeBox;
    
    JLabel[] labels = {new JLabel("Result Type"), new JLabel("Final Analysis")};
    JLabel[] countLabels = {new JLabel("Points"), new JLabel("0")};
    JLabel[] avgErrorLabels = {new JLabel("Avg Error %"), new JLabel("0")};
    JLabel[] maxErrorLabels = {new JLabel("Max Error %"), new JLabel("0")};
    JLabel[] kpLabels = {new JLabel("KP"), new JLabel("0")};
    Sysid app;

    public SysidResultPanel(Sysid app) {
        super(new GridLayout(nK + 7, 2, 5, 5));
        this.app = app;

        add(new JLabel("Mechanism Type:"));
        mechTypeBox = new JComboBox<>(LogReader.MechanismType.values());
        
        mechTypeBox.addActionListener(e -> {
            LogReader.MechanismType selectedType = (LogReader.MechanismType) mechTypeBox.getSelectedItem();
            Sysid.msg("Recalculating as " + selectedType + "...");
            app.analyzeFile(selectedType);
        });
        add(mechTypeBox);
        
        add(new JLabel(""));
        add(new JLabel(""));

        add(labels[0]); add(labels[1]);
        add(countLabels[0]); add(countLabels[1]);
        add(avgErrorLabels[0]); add(avgErrorLabels[1]);
        add(maxErrorLabels[0]); add(maxErrorLabels[1]);

        checkBoxes = new JCheckBox[nK];
        k = new JLabel[nK][1];
        for(int i = 0; i < nK; i++) {
            checkBoxes[i] = new JCheckBox(KTypes.values()[i].name());
            checkBoxes[i].setEnabled(false);
            checkBoxes[i].setSelected(true);
            add(checkBoxes[i]);
            
            k[i][0] = new JLabel("0");
            add(k[i][0]);
        }

        add(kpLabels[0]); add(kpLabels[1]);
    }
    
    public void updateDisplay(MotorData motorData) {
        if (motorData == null || motorData.sysidResult == null) {
            clearDisplay();
            return;
        }
        
        LogReader.SysIDResults result = motorData.sysidResult;
        LogReader.BucketResult bucket = result.finalFit;
        
        if(bucket != null) {
            k[KTypes.KS.ordinal()][0].setText(String.format("%7.5f", bucket.ks));
            k[KTypes.KV.ordinal()][0].setText(String.format("%7.5f", bucket.kv));
            k[KTypes.KA.ordinal()][0].setText(String.format("%7.5f", bucket.ka));
            k[KTypes.KG.ordinal()][0].setText(String.format("%7.5f", bucket.kg));
            
            countLabels[1].setText(Integer.toString(bucket.points));
            avgErrorLabels[1].setText(String.format("%4.2f%%", bucket.avgError*100));
            maxErrorLabels[1].setText(String.format("%4.2f%%", bucket.avgError*120));
            
            kpLabels[1].setText(String.format("%.5f", bucket.kp));
        } else {
            clearDisplay();
        }
        
        Sysid.msg("Display updated for " + motorData.name);
    }
    
    private void clearDisplay() {
        for(int i = 0; i < nK; i++) {
            k[i][0].setText("0");
        }
        countLabels[1].setText("0");
        avgErrorLabels[1].setText("0");
        maxErrorLabels[1].setText("0");
        kpLabels[1].setText("0");
    }
}

class MotorData {
    static List<MotorData> motors = new ArrayList<>();
    String name = "Motor";
    LogReader.SysIDResults sysidResult;

    @Override
    public String toString() {
        return name;
    }
}

enum KTypes {KS,KV,KA,KG}