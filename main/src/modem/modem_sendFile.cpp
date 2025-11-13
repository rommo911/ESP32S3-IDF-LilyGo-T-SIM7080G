
#include <Arduino.h>
#include "modem/modem.hpp"
#include <algorithm> // Include the <algorithm> header for the std::min function

namespace modem
{

    bool sendFileToModem(File file, const char *destinationfilename)
    {
        modem7080g.sendAT("+CFSTERM"); // Close FS in case it's still initialized
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("Failed to terminate file system");
            return false;
        }

        modem7080g.sendAT("+CFSINIT"); // Initialize FS
        if (modem7080g.waitResponse() != 1)
        {
            Serial.println("Failed to initialize file system");
            return false;
        }
        bool firstChunk = true;
        // Get the total size of the file
        size_t totalSize = file.size();
        size_t alreadySent = 0;
        // Loop for sending chunks
        const size_t bufferSize = totalSize + 50; // 10KB buffer size

        uint8_t *buffer = new uint8_t[bufferSize];

        bool success = true;
        while (totalSize > 0)
        {
            // Determine the size of the next chunk to send
            size_t chunkSize = std::min(totalSize, static_cast<size_t>(4096)); // Limit chunk size to 10,000 bytes
            // Prepare the file upload command
            String command = "+CFSWFILE=0,\"" +
                             String(destinationfilename != NULL ? destinationfilename : file.name()) +
                             "\"," + String(firstChunk ? 0 : 1) + "," + String(chunkSize) + ",4096";
            Serial.println(command);            // For reference
            modem7080g.sendAT(command.c_str()); // Send file upload command
            // if (modem7080g.waitResponse(30000UL, "ERROR") == 1) { // Wait for modem confirmation
            //     Serial.println("Modem did not respond with DOWNLOAD");
            //     return;
            // }

            // Write the chunk of data to the modem
            size_t bytesRead = file.read(buffer, std::min(chunkSize, bufferSize)); // Read chunkSize bytes from the file
            if (bytesRead > 0)
            {
                size_t bytesWritten = modem7080g.stream.write(buffer, bytesRead); // Write the read data to the modem's stream
                if (bytesWritten != bytesRead)
                {
                    Serial.println("Failed to write chunk to modem");
                    success = false;
                    break;
                }
                alreadySent += bytesWritten;
                totalSize -= bytesWritten;

                Serial.printf("Sent %d bytes, %d bytes remaining\n", bytesWritten, totalSize);
            }
            else
            {
                Serial.println("Failed to read chunk from file");
                success = false;
                break;
            }

            firstChunk = false; // Update the flag after the first chunk
        }
        delete[] buffer;
        if (success)
        {
            Serial.println("File upload completed");
            modem7080g.sendAT("+CFSTERM");
            if (modem7080g.waitResponse() != 1)
            {
                Serial.println("Failed to terminate file system after sending the file");
                return false;
            }
            return true;
        }
        else
        {
            Serial.println("File upload failed");
            return false;
        }
        // Terminate file system after sending the file
    }
} // namesapce