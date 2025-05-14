#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>

// OLED definitions
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Declare an SSD1306 display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Audio input pin
const int AUDIO_INPUT_PIN = A0;

// FFT definitions
const uint16_t samples = 128; // Number of samples for FFT (must be a power of 2)
const double samplingFrequency = 10000; // Sampling frequency (samples per second) - Adjust as needed
const float FREQ_PER_BIN = (double)samplingFrequency / samples; // Frequency resolution per bin (Hz)


// FFT objects
arduinoFFT FFT = arduinoFFT();
double vReal[samples];
double vImag[samples];

unsigned long newTime;
unsigned long oldTime;
double samplingPeriod;

// Array to store previous FFT magnitudes for smoothing
double prevMagnitudes[samples / 2] = {0};


// Global Variables for Peak Hold
// Array to store the peak magnitude for each frequency bin
double peakMagnitudes[samples / 2] = {0};
// Decay rate for the peak hold dots (0.0 to 1.0).
// 1.0 means no decay, 0.0 means instant decay.
// A value around 0.8 to 0.98 provides a visible peak hold effect.
const float peakDecayRate = 0.96;

// Global Variables for Variable Sensitivity
// Base scaling factor for mapping magnitudes to display height for higher frequencies.
// Adjust based on the typical max magnitude from your audio source and FFT.
// A smaller value increases sensitivity (bars get taller).
const double baseMagnitudeScale = 1500.0;

// The frequency (Hz) below which sensitivity starts decreasing.
// The transition will be smooth from 0Hz up to this frequency.
const float lowFreqRolloffFreq = 400.0;

// Variable for Display Height Adjustment
const int topMarginPixels = 0; // Number of pixels to leave blank at the top
const int effectiveDisplayHeight = SCREEN_HEIGHT - topMarginPixels; // The actual height used for drawing FFT

// Global Variables for Temporal Interpolation
// Array to store the current magnitude value being displayed for interpolation
double interpolatedMagnitudes[samples / 2] = {0};
// Factor for temporal interpolation (0.0 to 1.0).
// 1.0 means no interpolation (instant update), 0.0 means no movement.
// A value around 0.2 to 0.6 provides smooth movement.
const float interpolationFactor = 0.7; // Adjust this value for smoothness


// Helper function for mapping a double value with a double range to an integer range
// This is needed because standard map() works only with long integers.
int customMap(double value, double fromLow, double fromHigh, int toLow, int toHigh) {
    // Prevent division by zero and handle cases outside the source range
    if (fromHigh == fromLow) return toLow;
    // Ensure value is within the source range before mapping
    if (value <= fromLow) return toLow;
    if (value >= fromHigh) return toHigh; // Any value >= fromHigh maps to toHigh

    // Linear interpolation using double for precision
    return (int)((value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow);
}


void setup() {
    Serial.begin(115200);

    // Initialize I2C for OLED
    Wire.begin(4, 5); // SDA, SCL

    // Initialize OLED display
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;); // Don't proceed, loop forever
    }

    display.setRotation(2);
    // Clear the buffer
    display.clearDisplay();
    display.display();

    // Set up sampling period
    samplingPeriod = 1.0 / samplingFrequency;

    // Initialize magnitude arrays to zero
    for(int i = 0; i < samples / 2; i++) {
        prevMagnitudes[i] = 0;
        peakMagnitudes[i] = 0;
        interpolatedMagnitudes[i] = 0; // Initialize interpolated magnitudes
    }

    Serial.println("Setup complete. Starting FFT...");
    Serial.print("Frequency Resolution: ");
    Serial.print(FREQ_PER_BIN);
    Serial.println(" Hz per bin");
    Serial.print("Low Frequency Rolloff ends around bin: ");
    Serial.println(lowFreqRolloffFreq / FREQ_PER_BIN);
    Serial.print("Effective Display Height for FFT: ");
    Serial.print(effectiveDisplayHeight);
    Serial.println(" pixels");
    Serial.print("Base Magnitude Scale: ");
    Serial.println(baseMagnitudeScale);
}

void loop() {
    // Audio Sampling
    oldTime = micros();
    for (int i = 0; i < samples; i++) {
        newTime = micros();
        // Wait for the next sample time to maintain the sampling frequency
        while ((newTime - oldTime) < samplingPeriod * 1000000) {
            newTime = micros();
        }
        oldTime = newTime;

        // Read analog value from the audio input pin (0-4095 for ESP32 ADC)
        // Subtract midpoint to approximate DC removal during sampling
        vReal[i] = analogRead(AUDIO_INPUT_PIN) - 2048.0;
        vImag[i] = 0; // Initialize imaginary part to 0
    }

    // Perform FFT
    // DC Removal helps center the data around zero for FFT.
    FFT.DCRemoval(vReal, samples);

    // Apply a window function (Hamming recommended for reducing spectral leakage)
    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); // Compute FFT
    FFT.ComplexToMagnitude(vReal, vImag, samples); // Convert complex numbers to magnitudes (strength of each frequency)

    // Display on OLED
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    // Apply peak decay to all stored peak magnitudes BEFORE processing bins
    for (int i = 0; i < samples / 2; i++) {
        peakMagnitudes[i] *= peakDecayRate;
    }

    // Draw the smoothed FFT spectrum as bars and add peak dots
    // We display the first half of the FFT results (up to the Nyquist frequency).
    for (int i = 0; i < samples / 2; i++) {
        // Calculate the current magnitude for this frequency bin
        double currentMagnitude = vReal[i];

        // Calculate the horizontal position on the display for this frequency bin.
        // This maps the 64 bins across the 128 pixel width, drawing lines at even x positions (0, 2, 4...).
        int xPos = map(i, 0, samples / 2, 0, SCREEN_WIDTH);

        // Calculate Current Bin Magnitude Scale (for variable sensitivity)
        double currentBinMagnitudeScale;
        float binFreq = i * FREQ_PER_BIN; // Frequency of the current bin

        if (binFreq >= lowFreqRolloffFreq) {
            // For frequencies at or above the rolloff frequency, use the base sensitivity scale
            currentBinMagnitudeScale = baseMagnitudeScale;
        } else {
            currentBinMagnitudeScale = (baseMagnitudeScale * ( 1 + ( 3.5 * (lowFreqRolloffFreq - binFreq) / lowFreqRolloffFreq)));
        }

        // Bar Smoothing
        double targetMagnitude = (currentMagnitude + prevMagnitudes[i]) / 2.0; // Target for interpolation

        // Store the current magnitude for the next smoothing calculation
        prevMagnitudes[i] = currentMagnitude;

        // Temporal Interpolation
        // Smoothly move the displayed magnitude towards the target magnitude
        interpolatedMagnitudes[i] = interpolatedMagnitudes[i] * (1.0 - interpolationFactor) + targetMagnitude * interpolationFactor;

        // Peak Hold Update
        // Update peak using the *smoothed* magnitude (or could use currentMagnitude for sharper peaks)
        // Using targetMagnitude (smoothed) for peak detection
        if (targetMagnitude > peakMagnitudes[i]) {
            peakMagnitudes[i] = targetMagnitude;
        }
        // Decay was applied to all peaks before this loop

        // Calculate Heights for Drawing
        // Scale the interpolated magnitude to fit the effective display height (leaving space at the top).
        // Use the bin-specific magnitude scale (currentBinMagnitudeScale) for mapping.
        int barHeight = customMap(interpolatedMagnitudes[i], 0.0, currentBinMagnitudeScale, 0, effectiveDisplayHeight);
        // Ensure height is within bounds of the effective drawing area
        if (barHeight > effectiveDisplayHeight) barHeight = effectiveDisplayHeight;
        else if (barHeight < 0) barHeight = 0;

        // Scale the peak magnitude to fit the effective display height (leaving space at the top).
        // Use the bin-specific magnitude scale (currentBinMagnitudeScale) for mapping.
        int peakHeight = customMap(peakMagnitudes[i], 0.0, currentBinMagnitudeScale, 0, effectiveDisplayHeight);
        // Ensure height is within bounds of the effective drawing area
        if (peakHeight > effectiveDisplayHeight) peakHeight = effectiveDisplayHeight;
        else if (peakHeight < 0) peakHeight = 0;

        // Draw Bar and Peak
        // Draw a vertical line (bar) for this frequency bin.
        // Draw the bar from the bottom of the screen upwards, respecting the top margin.
        display.drawLine(xPos, SCREEN_HEIGHT, xPos, SCREEN_HEIGHT - barHeight, SSD1306_WHITE);

        // Draw the peak hold dot.
        // Draw a single pixel at the peak height for this column, respecting the top margin.
        display.drawPixel(xPos, SCREEN_HEIGHT - peakHeight, SSD1306_WHITE);
    }

    // Update the physical display with the contents of the buffer
    display.display();

    // Small delay before the next loop iteration - controls frame rate
    delay(1);
}
