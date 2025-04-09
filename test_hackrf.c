#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <libhackrf/hackrf.h>
#include <math.h>
#include <stdbool.h>
#include <fftw3.h>

// Global variables
static hackrf_device* device = NULL;
static volatile bool hackrf_running = true;
static double signal_power_db = -999.0; // Global variable to store signal power
static uint32_t center_frequency = 2700000000; // Default center frequency (2.7 GHz)

// Configuration
#define SAMPLE_RATE 20000000 // 20 MHz sample rate
#define BANDWIDTH 100000    // 2 MHz bandwidth
#define FFT_SIZE 1024        // FFT size

// Function to calculate power in dB
float logPower(fftwf_complex in, float scale) {
    float re = in[0] * scale;
    float im = in[1] * scale;
    float magsq = re * re + im * im;
    return (float)(10.0f * log10(magsq));
}

// Callback function to process HackRF data
int HackRFSweepCallback(hackrf_transfer* transfer) {
    static fftwf_complex fft_in[FFT_SIZE];
    static fftwf_complex fft_out[FFT_SIZE];
    static fftwf_plan fft_plan = NULL;

    if (!fft_plan) {
        fft_plan = fftwf_plan_dft_1d(FFT_SIZE, fft_in, fft_out, FFTW_FORWARD, FFTW_ESTIMATE);
    }

    // Copy I/Q data into FFT input buffer
    for (int i = 0; i < FFT_SIZE; i++) {
        fft_in[i][0] = transfer->buffer[2 * i] / 128.0f;     // I (real part)
        fft_in[i][1] = transfer->buffer[2 * i + 1] / 128.0f; // Q (imaginary part)
    }

    // Perform FFT
    fftwf_execute(fft_plan);

    // Calculate power for each frequency bin
    float bin_width = (float)SAMPLE_RATE / FFT_SIZE;
    float start_freq = center_frequency - BANDWIDTH / 2;
    float end_freq = center_frequency + BANDWIDTH / 2;
    float total_power = 0.0f;
    int count = 0;

    for (int i = 0; i < FFT_SIZE; i++) {
        float freq = center_frequency + (i - FFT_SIZE / 2) * bin_width;
        if (freq >= start_freq && freq <= end_freq) {
            total_power += logPower(fft_out[i], 1.0f / FFT_SIZE);
            count++;
        }
    }

    // Calculate average power
    if (count > 0) {
        signal_power_db = total_power / count;
    } else {
        signal_power_db = -999.0; // No valid data
    }

    return 0;
}

// Initialize HackRF for sweeping
int InitHackRFSweep(uint32_t freq) {
    int result;

    // Initialize HackRF
    result = hackrf_init();
    if (result != HACKRF_SUCCESS) {
        printf("Failed to initialize HackRF: %s\n", hackrf_error_name(result));
        return -1;
    }

    // Open HackRF device
    result = hackrf_open(&device);
    if (result != HACKRF_SUCCESS) {
        printf("Failed to open HackRF device: %s\n", hackrf_error_name(result));
        hackrf_exit();
        return -1;
    }

    // Set sample rate
    result = hackrf_set_sample_rate(device, SAMPLE_RATE);
    if (result != HACKRF_SUCCESS) {
        printf("Failed to set sample rate: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return -1;
    }

    // Set center frequency
    center_frequency = freq;
    result = hackrf_set_freq(device, center_frequency);
    if (result != HACKRF_SUCCESS) {
        printf("Failed to set center frequency: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return -1;
    }

    // Set LNA gain to 40 dB
    result = hackrf_set_lna_gain(device, 40);
    if (result != HACKRF_SUCCESS) {
        printf("Failed to set LNA gain: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return -1;
    }

    // Start receiving data
    result = hackrf_start_rx(device, HackRFSweepCallback, NULL);
    if (result != HACKRF_SUCCESS) {
        printf("Failed to start HackRF sweep: %s\n", hackrf_error_name(result));
        hackrf_close(device);
        hackrf_exit();
        return -1;
    }

    printf("HackRF initialized for 2 MHz bandwidth centered at %.2f GHz with LNA gain set to 40 dB.\n",
           center_frequency / 1e9);
    return 0;
}

// Cleanup HackRF resources
void CleanupHackRF() {
    hackrf_running = false; // Stop the sweep
    if (device) {
        hackrf_stop_rx(device);
        hackrf_close(device);
    }
    hackrf_exit();
    printf("HackRF resources cleaned up.\n");
}

int main(int argc, char* argv[]) {
    uint32_t freq = 2700000000; // Default center frequency (2.7 GHz)

    // Check if the user provided a center frequency as an argument
    if (argc > 1) {
        freq = strtoul(argv[1], NULL, 10); // Convert argument to unsigned long
        if (freq < 1000000 || freq > 6000000000) { // Validate range (1 MHz to 6 GHz)
            printf("Invalid center frequency. Please specify a value between 1 MHz and 6 GHz.\n");
            return -1;
        }
    }

    printf("Starting HackRF test...\n");

    // Initialize HackRF
    if (InitHackRFSweep(freq) != 0) {
        printf("HackRF initialization failed. Exiting.\n");
        return -1;
    }

    // Run the sweep for 10 seconds
    for (int i = 0; i < 10; i++) {
        printf("Signal power in 2 MHz bandwidth centered at %.2f GHz: %.2f dB\n",
               freq / 1e9, signal_power_db);
        sleep(1); // Wait for 1 second
    }

    // Cleanup HackRF
    CleanupHackRF();

    printf("HackRF test completed.\n");
    return 0;
}