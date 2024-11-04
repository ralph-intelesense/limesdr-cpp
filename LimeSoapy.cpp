#include <cstdio>	//stdandard output
#include <cstdlib>

// Compile with: g++ LimeSoapy.cpp -o LimeSoapy -lSoapySDR
// #include <SoapySDR/Device.hpp>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Types.hpp>
#include <SoapySDR/Formats.hpp>
#include <string>	// std::string
#include <vector>	// std::vector<...>
#include <map>		// std::map< ... , ... >

#include <iostream>

#include <complex>
#include <chrono>
#include <thread>

class SDRDevice {
public:
    SDRDevice(double sampleRate, double frequency, double gain)
        : sampleRate(sampleRate), frequency(frequency), gain(gain), device(nullptr), rxStream(nullptr) {}

    // Initialize the SDR device
    bool initialize() {
        // Discover available devices
        auto devices = SoapySDR::Device::enumerate();
        if (devices.empty()) {
            std::cerr << "No SDR devices found!" << std::endl;
            return false;
        }
    
        // Open the first device found
        device = SoapySDR::Device::make(devices[1]);
        if (!device) {
            std::cerr << "Failed to create SDR device!" << std::endl;
            return false;
        }

        // Configure device parameters
        device->setSampleRate(SOAPY_SDR_RX, 0, sampleRate);
        device->setFrequency(SOAPY_SDR_RX, 0, frequency);
        device->setGain(SOAPY_SDR_RX, 0, gain);

        return true;
    }

    // Set up the RX stream
    bool setupStream() {
        std::vector<size_t> channels = {0}; // Use channel 0
        rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, channels);
        if (rxStream == nullptr) {
            std::cerr << "Failed to setup RX stream!" << std::endl;
            return false;
        }
        return true;
    }

    // Activate the stream
    void startStreaming() {
        if (device && rxStream) {
            device->activateStream(rxStream);
        }
    }

    // Read samples from the device
    int readSamples(std::vector<std::complex<float>>& buffer, int timeoutUs = int(1e5)) {
        if (!device || !rxStream) return -1;

        void* buffs[] = { buffer.data() };
        int flags;
        long long timeNs;

        int ret = device->readStream(rxStream, buffs, buffer.size(), flags, timeNs, timeoutUs);
        if (ret < 0) {
            printf("Error reading stream!");
            // std::cerr << "Error reading samples: " << SoapySDR::errToStr(ret) << std::endl;
        }
        return ret;
    }

    // Stop the stream
    void stopStreaming() {
        if (device && rxStream) {
            device->deactivateStream(rxStream);
        }
    }

    // Clean up the stream and device
    void cleanup() {
        if (device) {
            if (rxStream) {
                device->closeStream(rxStream);
                rxStream = nullptr;
            }
            SoapySDR::Device::unmake(device);
            device = nullptr;
        }
    }

    // Destructor to ensure cleanup
    ~SDRDevice() {
        cleanup();
    }

private:
    double sampleRate;
    double frequency;
    double gain;
    SoapySDR::Device* device;
    SoapySDR::Stream* rxStream;
};

int main() {
    // Define SDR parameters
    double sampleRate = 10e6;  // 1 MS/s
    double frequency = 413e6; // 100 MHz
    double gain = 30.0;       // 30 dB

    // Create and initialize SDR device
    SDRDevice sdr(sampleRate, frequency, gain);
    if (!sdr.initialize()) {
        return EXIT_FAILURE;
    }

    // Setup and start the RX stream
    if (!sdr.setupStream()) {
        return EXIT_FAILURE;
    }
    sdr.startStreaming();

    // Buffer to hold samples
    const int numSamples = 1024;
    std::vector<std::complex<float>> buffer(numSamples);

    // Read and process samples
    for (int i = 0; i < 10; i++) {
        int numRead = sdr.readSamples(buffer);
        if (numRead > 0) {
            std::cout << "Read " << numRead << " samples:" << std::endl;
            // for (int j = 0; j < numRead; ++j) {
            //     std::cout << "Sample " << j << ": "
            //               << "Real = " << buffer[j].real() << ", "
            //               << "Imag = " << buffer[j].imag() << std::endl;
            // }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop streaming and clean up
    sdr.stopStreaming();
    sdr.cleanup();

    return EXIT_SUCCESS;
}

