
#include <Arduino.h>
#include "modem/modem.hpp"
#include <algorithm> // Include the <algorithm> header for the std::min function

namespace modem
{
    String sendHttpGET(const char* url)
    {
        TinyGsmClient client(modem7080g, 0);
        const int port = 80;
        
        // Parse URL to get server and resource
        String server = url;
        String resource = "/";
        int pathIndex = server.indexOf('/', 7); // Skip "http://"
        if (pathIndex > 0) {
            server = server.substring(7, pathIndex);
            resource = url + pathIndex;
        }

        String response = "";
        if (!client.connect(server.c_str(), port)) {
            return "Connection failed";
        }

        // Make HTTP GET request
        client.print(String("GET ") + resource + " HTTP/1.1\r\n");
        client.print(String("Host: ") + server + "\r\n");
        client.print("Connection: close\r\n\r\n");

        // Wait for data
        uint32_t start = millis();
        while (client.connected() && !client.available() && millis() - start < 30000L) {
            delay(100);
        }

        // Read response
        while (client.connected() && millis() - start < 10000L) {
            while (client.available()) {
                response += (char)client.read();
            }
        }

        client.stop();
        return response;
    }

    String sendHttpsGET(const char* url)
    {
        TinyGsmClientSecure client(modem7080g, 0);
        const int port = 443;
        
        // Parse URL to get server and resource
        String server = url;
        String resource = "/";
        int pathIndex = server.indexOf('/', 8); // Skip "https://"
        if (pathIndex > 0) {
            server = server.substring(8, pathIndex);
            resource = url + pathIndex;
        }

        String response = "";
        if (!client.connect(server.c_str(), port)) {
            return "Secure connection failed";
        }

        // Make HTTPS GET request
        client.print(String("GET ") + resource + " HTTP/1.1\r\n");
        client.print(String("Host: ") + server + "\r\n");
        client.print("Connection: close\r\n\r\n");

        // Wait for data
        uint32_t start = millis();
        while (client.connected() && !client.available() && millis() - start < 30000L) {
            delay(100);
        }

        // Read response
        while (client.connected() && millis() - start < 10000L) {
            while (client.available()) {
                response += (char)client.read();
            }
        }

        client.stop();
        return response;
    }

    String SendhttpPOST(const char* url, const char* data)
    {
        TinyGsmClient client(modem7080g, 0);
        const int port = 80;
        
        // Parse URL to get server and resource
        String server = url;
        String resource = "/";
        int pathIndex = server.indexOf('/', 7); // Skip "http://"
        if (pathIndex > 0) {
            server = server.substring(7, pathIndex);
            resource = url + pathIndex;
        }

        String response = "";
        if (!client.connect(server.c_str(), port)) {
            return "Connection failed";
        }

        // Calculate content length
        int contentLength = strlen(data);

        // Make HTTP POST request
        client.print(String("POST ") + resource + " HTTP/1.1\r\n");
        client.print(String("Host: ") + server + "\r\n");
        client.print("Content-Type: application/x-www-form-urlencoded\r\n");
        client.print(String("Content-Length: ") + contentLength + "\r\n");
        client.print("Connection: close\r\n\r\n");
        client.print(data);

        // Wait for data
        uint32_t start = millis();
        while (client.connected() && !client.available() && millis() - start < 30000L) {
            delay(100);
        }

        // Read response
        while (client.connected() && millis() - start < 10000L) {
            while (client.available()) {
                response += (char)client.read();
            }
        }

        client.stop();
        return response;
    }

    String SendHttpsPOST(const char* url, const char* data)
    {
        TinyGsmClientSecure client(modem7080g, 0);
        const int port = 443;
        
        // Parse URL to get server and resource
        String server = url;
        String resource = "/";
        int pathIndex = server.indexOf('/', 8); // Skip "https://"
        if (pathIndex > 0) {
            server = server.substring(8, pathIndex);
            resource = url + pathIndex;
        }

        String response = "";
        if (!client.connect(server.c_str(), port)) {
            return "Secure connection failed";
        }

        // Calculate content length
        int contentLength = strlen(data);

        // Make HTTPS POST request
        client.print(String("POST ") + resource + " HTTP/1.1\r\n");
        client.print(String("Host: ") + server + "\r\n");
        client.print("Content-Type: application/x-www-form-urlencoded\r\n");
        client.print(String("Content-Length: ") + contentLength + "\r\n");
        client.print("Connection: close\r\n\r\n");
        client.print(data);

        // Wait for data
        uint32_t start = millis();
        while (client.connected() && !client.available() && millis() - start < 30000L) {
            delay(100);
        }

        // Read response
        while (client.connected() && millis() - start < 10000L) {
            while (client.available()) {
                response += (char)client.read();
            }
        }

        client.stop();
        return response;
    }
} // namespace