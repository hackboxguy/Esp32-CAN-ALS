CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -O2 -pthread
TARGET = calibrate_lux_sensor
SOURCE = calibrate_lux_sensor.cpp

$(TARGET): $(SOURCE)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(SOURCE)

clean:
	rm -f $(TARGET)

install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin/

.PHONY: clean install
