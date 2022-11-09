#include "WIFI_comms.h"
#ifdef USE_WIFI_COMMUNICATION

payload_t data;
// uint8_t sendTo[] = {0xE8, 0xDB, 0x84, 0x94, 0x6B, 0x78};
uint8_t sendTo[] = {0xBC, 0xFF, 0x4D, 0x18, 0xF5, 0xF6};

wifiServer::wifiServer() : server(80), webSocket(81), message(""), dB(0) {}

bool wifiServer::init(const char *ssid /*=WIFI_SSID*/,
                      const char *passward /*=WIFI_PASSWARD*/)
{
#ifdef AP_AS_SERVER      // Access point as server (Sky recommanded)
    WiFi.mode(WIFI_AP);  // Set access point ssid and passward
    if (!WiFi.softAP(ssid, passward)) {
        Serial.println("Set ssid and passward failed");
        return false;
    }
    WiFi.hostname(WIFI_HOST_NAME);  // Set hostname as local host name
    Serial.print(String(WIFI_HOST_NAME) + ": ");
    Serial.println(WiFi.softAPIP());
    Serial.println(String("Or you can use http://") + WIFI_HOST_NAME + "/");
#elif defined(STA_AS_SERVER)  // Station as server (Ground recommanded)
    WiFi.mode(WIFI_STA);
    if (!WiFi.begin(ssid, passward)) {
        Serial.println("Set ssid and passward failed");
        return false;
    }
#endif
    Serial.println(String("MAC Address: ") + WiFi.macAddress());

    server.onNotFound(
        [=]() {  //[=] lambda expression calling all variables by value
            // server.uri() --> server get request path from client
            if (!handleFileRead(server.uri())) {
                server.send(404, "text/plain", "FileNotFound");
            } else {
                server.send(200);  // 200 means OK
            }
        });

    // List current file on board by using "/list" link
    server.on("/list", [=]() { server.send(200, "text/plain", listFile()); });

    // WebSocketEvent waits for webSocket client to send command
    webSocket.onEvent(std::bind(&wifiServer::webSocketEvent, this,
                                std::placeholders::_1, std::placeholders::_2,
                                std::placeholders::_3, std::placeholders::_4));

    // Start wesocket and server service
    webSocket.begin();
    server.begin();

#ifdef ESP_NOW
    // Init ESP-NOW
    if (esp_now_init() != 0) {
        Serial.println("Error initializing ESP-NOW");
        return false;
    }
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(onDataSend);
    esp_now_register_recv_cb(onDataRecv);

    // Register peer
    esp_now_add_peer(sendTo, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
#endif

    return true;  // If all things operate successfully
}

bool wifiServer::handleFileRead(String path)
{
    Serial.println("handleFileRead: " + path);
    // If server request end with "/", then auto direct to "index.html"
    if (path.endsWith("/")) {
        path += "index.html";
    }
    String contentType = getContentType(path);  // Get file type
    String pathWithGz = path + ".gz";
    if (filesystem->exists(pathWithGz) || filesystem->exists(path)) {
        if (filesystem->exists(pathWithGz)) {
            path += ".gz";
        }
        File file = filesystem->open(path, "r");  // Open file by path
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.streamFile(file, contentType);  // stream the file to all clients
        file.close();
        return true;
    }
    return false;
}

String wifiServer::getContentType(String filename)
{  // identify sub-file-name and return
    // the corespond content-type
    if (server.hasArg("download")) {
        // if server request has argument "download" or not
        return "application/octet-stream";
    } else if (filename.endsWith(".htm")) {
        return "text/html";
    } else if (filename.endsWith(".html")) {
        return "text/html";
    } else if (filename.endsWith(".css")) {
        return "text/css";
    } else if (filename.endsWith(".js")) {
        return "application/javascript";
    } else if (filename.endsWith(".png")) {
        return "image/png";
    } else if (filename.endsWith(".gif")) {
        return "image/gif";
    } else if (filename.endsWith(".jpg")) {
        return "image/jpeg";
    } else if (filename.endsWith(".ico")) {
        return "image/x-icon";
    } else if (filename.endsWith(".xml")) {
        return "text/xml";
    } else if (filename.endsWith(".pdf")) {
        return "application/x-pdf";
    } else if (filename.endsWith(".zip")) {
        return "application/x-zip";
    } else if (filename.endsWith(".gz")) {
        return "application/x-gzip";
    }
    return "text/plain";
}

void wifiServer::webSocketEvent(uint8_t num,
                                WStype_t type,
                                uint8_t *payload,
                                size_t length)
{
    String device;
    switch (type) {
    // If websocket is disconnected
    case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\n", num);
        device = String(num) + " disconnected";
        webSocket.broadcastTXT(device);
        message = "disconnected";
        break;
    // If websocket is connected
    case WStype_CONNECTED: {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0],
                      ip[1], ip[2], ip[3], payload);

        // Send message to client
        device = String(num) + " has connected";
        webSocket.broadcastTXT(device);
        message = "connected";
    } break;
    // If websocket get text message (Use this as command)
    case WStype_TEXT:
        Serial.printf("[%u] : %s\n", num, payload);
        message = (const char *) payload;

        if (message[0] == 'w') {
            dB = message.substring(2).toInt();
        }
        // Send message to client
        // webSocket.sendTXT(num, "message here");

        // Send data to all connected clients
        // webSocket.broadcastTXT("message here");
        break;
    // If websocket get binary message
    case WStype_BIN:
        Serial.printf("[%u] get binary length: %u\n", num, length);
        hexdump(payload, length);

        // Send message to client
        // webSocket.sendBIN(num, payload, length);
        break;
    }
}

// This section is for sending data to client
// cleanMsg is default true, it will auto clean the message
// Turn it off if you don't want message be cleaned after calling it
bool wifiServer::wifi_send(uint8_t num, String payload, bool cleanMsg)
{
#ifdef USE_WIFI_COMMUNICATION
    bool success = webSocket.sendTXT(num, payload);
    if (success && cleanMsg)
        message = "";
    return success;
#endif
}
bool wifiServer::wifi_send(uint8_t num, const char *payload, bool cleanMsg)
{
#ifdef USE_WIFI_COMMUNICATION
    bool success = webSocket.sendTXT(num, payload);
    if (success && cleanMsg)
        message = "";
    return success;
#endif
}

bool wifiServer::wifi_broadcast(const String &payload, bool cleanMsg)
{
    return wifi_broadcast(payload.c_str(), cleanMsg);
}

bool wifiServer::wifi_broadcast(const char *payload, bool cleanMsg)
{
    bool success = false;
#ifdef USE_WIFI_COMMUNICATION
    success |= webSocket.broadcastTXT(payload);
#endif
#ifdef ESP_NOW
    success |= esp_now_send(NULL, (u8 *) payload, sizeof(payload));
#endif
    if (success && cleanMsg)
        message = "";
    return success;
}

void wifiServer::loop()
{
    server.handleClient();  // Loop for server
    webSocket.loop();       // Loop for websocket
    MDNS.update();          // For muiltipule clients to connect
}

#ifdef ESP_NOW
void onDataSend(uint8_t *mac_addr, uint8_t status)
{
    Serial.print("Last Packet Send Status: ");
    if (status == 0) {
        Serial.println("Delivery success");
    } else {
        Serial.println("Delivery fail");
    }
}

void onDataRecv(uint8_t *mac_addr, uint8_t *payload, uint8_t length)
{
    memcpy(&data, payload, sizeof(data));
    Serial.print("Bytes received: ");
    Serial.println(length);
    Serial.print("Char: ");
    Serial.println(data.message);
    Serial.println();
}
#endif

#endif