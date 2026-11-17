#pragma once

// Starts the network task on core 0. It runs WiFi management (access point
// only, or saved networks with access point fallback - see WifiManager.h),
// the web page (with a captive portal on the AP side), the WebSocket at /ws,
// the recording download at /capture.csv, mDNS, ArduinoOTA and the battery
// monitor.
//
// WebSocket protocol (JSON text frames):
//   server -> page   {"type":"info", "name", "hostname", "suffix", "apPassword", "build"}
//                    {"type":"schema", "params":[{key,label,group,min,max,step,default,reboot,help,fine,unused},...]}
//                    {"type":"settings", "values":{key:value,...}, "dirty":bool}
//                    {"type":"telemetry", ...}           every telemetryMs
//                    {"type":"wifi", "status":{...}, "saved":[ssid,...], "max":n,
//                     "scanning":bool, "networks":[{ssid,rssi,open,saved},...]}
//                    {"type":"log", "boot", "lines":[{seq,ms,text},...]}
//                    {"type":"notice", "text":"...", "ok":bool}
//   page -> server   {"cmd":"set", "key":"K1", "value":-8}
//                    {"cmd":"pause", "paused":true}
//                    {"cmd":"joystick", "x":0.1, "y":-0.5}     (re-sent every 100 ms while held)
//                    {"cmd":"save"} | {"cmd":"load"} | {"cmd":"defaults"}
//                    {"cmd":"capture", "seconds":10, "trigger":"now"|"balance"} | {"cmd":"captureCancel"}
//                    {"cmd":"motorRetry"}
//                    {"cmd":"wifiScan"} | {"cmd":"wifiConnect"} | {"cmd":"apOnly", "on":true}
//                    {"cmd":"wifiAdd", "ssid":"...", "password":"..."}
//                    {"cmd":"wifiRemove", "ssid":"..."}
// Saved WiFi passwords are never sent to the page.
void startWebInterface();

// "BalanceBot-3f2a1c" (the access point SSID), used to name the robot.
const char *robotName();
