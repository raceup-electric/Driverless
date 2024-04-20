#include <CppLinuxSerial/SerialPort.hpp>
#include "GPS.h"
#include <string>

/* Usage:
Compile using:
g++ -o reader gps_reader.cpp GPS.c -lCppLinuxSerial -lstdc++

Run Using:
sudo ./reader
*/

GPS gps;
char msg[80];

using namespace mn::CppLinuxSerial;

int main()
{
    SerialPort serialPort("/dev/ttyUSB0", BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(10000); // Block for up to 100ms to receive data
    serialPort.Open();

    while (true)
    {
        std::string readData;
        serialPort.Read(readData);
        std::cout << "Read data = \"" << readData << "\"" << std::endl;

        const char *msg = readData.c_str();
        if (parse_NMEA_buffer((char *)msg, &gps))
        {
            std::cout << std::to_string((int)gps.time.day) << " - "
                      << std::to_string((int)gps.time.month) << " - "
                      << std::to_string(gps.time.year) << std::endl;

            std::cout << std::to_string((int)gps.time.hour) << ":"
                      << std::to_string((int)gps.time.minute) << ":"
                      << std::to_string(gps.time.second) << std::endl;

            std::cout << gps.velocity << std::endl;
            std::cout << gps.cog << std::endl;

            std::cout << gps.latitude.grad << " "
                      << std::to_string(gps.latitude.minutes) << " "
                      << gps.latitude.direction << std::endl;

            std::cout << gps.longitude.grad << " "
                      << std::to_string(gps.longitude.minutes) << " "
                      << gps.longitude.direction << std::endl;

            std::cout << "\n";
        }
        else
        {
            std::cout << "Error not enough data!!!!!!!!!\n";
        }
    }
    // Close the serial port
    // serialPort.Close();
}