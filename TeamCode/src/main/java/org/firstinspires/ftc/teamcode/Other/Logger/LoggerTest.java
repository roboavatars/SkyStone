package org.firstinspires.ftc.teamcode.Other.Logger;

public class LoggerTest {

    public static void main(String[] args) {
        Logger log = new Logger();
        log.addEntry("Test", "9876");
        log.addEntry("Test", "234");
        log.addEntry("Status", "Configured");
        log.addEntry("Location", "\"76,234\"");
    }
}
