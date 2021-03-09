package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public class PacketManager extends TelemetryPacket{

    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    public PacketManager(FtcDashboard dashboard) {
        this.dashboard = dashboard;
        packet = new TelemetryPacket();
    }

    public void put(String key, Object value){
        packet.put(key, value);
    }

    public void send(){
        dashboard.sendTelemetryPacket(packet);
    }
}
