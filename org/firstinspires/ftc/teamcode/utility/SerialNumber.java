/*
 *  This class provides a way to have a serial number that increments over time.
 *  This is primarily intended for the Data Logger,
 *  to have a way to have a serial number that automatically increments each time an OpMode is run.
*/
package org.firstinspires.ftc.teamcode.utility;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SerialNumber {
    // Standard path of Datalogs folder
    private String directoryPath = "/sdcard/FIRST/java/src/Datalogs";

    private Telemetry telemetry;        // for debugging

    SerialNumber(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    // Get the current serial number
    // This can return the same value repeatedly
    public int getSerialNumber() {
        int result = getSerialNumberFromFile();
        while (result > 9999) {
            result -= 10000;
        }
        return result;
    }

    // Increment the serial number
    public void incrementSerialNumber() {
        int lastUsedSerialNumber = getSerialNumber();

        String filePathName = serialNumberFileName();
        try (PrintWriter out = new PrintWriter(filePathName)) {
            out.println(String.valueOf(lastUsedSerialNumber + 1));
        }
        catch (IOException e)
        {
            return;
        }
    }

    // Read the serial number from the file, and convert it to integer.
    // If the file does not exist, or can't be read, use 0.
    // If an Exception occurs, use an exception code.
    private int getSerialNumberFromFile() {
        String filePathName = serialNumberFileName();

        String fileContent = "0";
        try
        {
            // Ensure file exists
            File serialNumberFile = new File(filePathName);
            serialNumberFile.createNewFile(); // if file already exists will do nothing

            // Read content of file; should consist of one line of text with a number in string form
            StringBuilder contentBuilder = new StringBuilder();
            FileReader fileReader = new FileReader(filePathName);
            try (BufferedReader br = new BufferedReader(fileReader)) {
                String firstLine = br.readLine();
                if (firstLine != null) {
                    contentBuilder.append(firstLine);
                    fileContent = contentBuilder.toString();
                }
            } catch (IOException iox) {
                telemetry.addData("DataLogger IOException", iox.getMessage());
                return 100;
            }
            telemetry.addData("fileContent", fileContent);
        }
        catch (Exception x) {
            telemetry.addData("DataLogger Exception", x.getMessage());
            return 200;
        }

        // Convert string in file to integer
        int result = 0;
        try {
            result = Integer.parseInt(fileContent);
        } catch (NumberFormatException nfx) {
            telemetry.addData("DataLogger NumberFormatException", nfx.getMessage());
            return 300;
        }

        return result;
    }

    private String serialNumberFileName() {
        return directoryPath + "/SerialNumber.txt";
    }
}
