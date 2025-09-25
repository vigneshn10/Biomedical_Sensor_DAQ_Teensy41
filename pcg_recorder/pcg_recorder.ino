#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// Audio system objects
AudioInputAnalog mic0(A2);
AudioFilterBiquad bpFilter0;
AudioRecordQueue recorder;
AudioConnection patchCord1(mic0, bpFilter0);
AudioConnection patchCord2(bpFilter0, recorder);

File audioFile;
unsigned long totalAudioBytes = 0;
bool isRecording = true;

String filename;

// Function to generate next available filename like RECORD001.WAV
String generateNextFilename() {
  int fileIndex = 1;
  String name;

  do {
    char buffer[20];
    sprintf(buffer, "RECORD%03d.WAV", fileIndex++);
    name = String(buffer);
  } while (SD.exists(name.c_str()));

  return name;
}

void setup() {
  Serial.begin(115200);
  AudioMemory(16);

  // Configure bandpass filter (optional)
  // bpFilter0.setBandpass(0, 200.0, 1.5);
  bpFilter0.setBandpass(0, 80.0, 1.2);
  bpFilter0.setBandpass(1, 80.0, 1.2);  // Apply a second bandpass to same channel



  // Initialize SD card
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    return;
  }

  // Generate unique filename
  filename = generateNextFilename();
  Serial.print("Saving to: ");
  Serial.println(filename);

  // Create and open the WAV file
  audioFile = SD.open(filename.c_str(), FILE_WRITE);
  if (!audioFile) {
    Serial.println("Failed to create audio file");
    return;
  }

  // Write placeholder WAV header
  writeWavHeader();

  // Start recording
  recorder.begin();
}

void loop() {
  // Continuously write buffered audio
  if (isRecording && recorder.available() > 0) {
    byte *data = (byte *)recorder.readBuffer();
    audioFile.write(data, 256);
    totalAudioBytes += 256;
    recorder.freeBuffer();
  }

  // Stop after 10 seconds (optional)
  if (millis() > 10000 && isRecording) {
    isRecording = false;
    recorder.end();
    updateWavHeader();
    audioFile.close();
    Serial.println("Recording complete.");
  }
}

void writeWavHeader() {
  byte header[44];

  // RIFF chunk
  memcpy(header, "RIFF", 4);
  *(uint32_t *)(header + 4) = 0;  // Placeholder for total file size
  memcpy(header + 8, "WAVE", 4);

  // fmt subchunk
  memcpy(header + 12, "fmt ", 4);
  *(uint32_t *)(header + 16) = 16;         // PCM header size
  *(uint16_t *)(header + 20) = 1;          // Audio format = PCM
  *(uint16_t *)(header + 22) = 1;          // Mono
  *(uint32_t *)(header + 24) = 44100;      // Sample rate
  *(uint32_t *)(header + 28) = 44100 * 2;  // Byte rate
  *(uint16_t *)(header + 32) = 2;          // Block align
  *(uint16_t *)(header + 34) = 16;         // Bits per sample

  // data subchunk
  memcpy(header + 36, "data", 4);
  *(uint32_t *)(header + 40) = 0;  // Placeholder for data size

  audioFile.write(header, 44);
}

void updateWavHeader() {
  // Update file size and data size fields in header
  audioFile.seek(4);
  uint32_t fileSize = totalAudioBytes + 36;
  audioFile.write((uint8_t *)&fileSize, 4);

  audioFile.seek(40);
  audioFile.write((uint8_t *)&totalAudioBytes, 4);
}
