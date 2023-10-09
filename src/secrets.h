// #define SECRET_SSID "mfpguest_2.4Ghz@unifi"  // replace MySSID with your WiFi network name
// #define SECRET_SSID "mfpguest_2.4Ghz
// #define SECRET_SSID "GFguestwifi_AP"
#define SECRET_SSID "MFPGuestWifi_FTY1"  // replace MySSID with your WiFi network name
#define SECRET_PASS "Pr0@#17774$$M@f1"
// #define SECRET_SSID "GFguestwifi_2.4ghz@unifi" 
// #define SECRET_PASS "GF@9300@a"
// #define SECRET_SSID "Pez Gordo" 
// #define SECRET_PASS "SardinaMacarena2021"
// #define SECRET_SSID "Maxis_9FF4D2"
// #define SECRET_PASS "35588590"
// #define SECRET_SSID "TP-Link_EE55"
// #define SECRET_PASS "42047483"
#define HOST_NAME "esp32"
// #define IP_ADDRESS "192.168.1.160" //previous ip : 10,0,3,21, replace that ip to match the IP of the device where the broker is running
// #define IP_ADDRESS "172.16.30.140" //previous ip : 10,0,3,21, replace that ip to match the IP of the device where the broker is running
// #define IP_ADDRESS "192.168.2.219" //previous ip : 10,0,3,21, replace that ip to match the IP of the device where the broker is running
// #define IP_ADDRESS "192.168.0.117" // TP Link
#define IP_ADDRESS "192.168.2.219" // FTY1

#define PORT      1883
#define USERNAME  "MDuino01"  //name of the device on to the broker
#define PASS      ""              // optional psw
#define TOPIC     "home/bedroom/temperature" //topic to publish to