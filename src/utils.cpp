/**
 * @file utils.cpp
 * @brief Utility function implementations
 */

#include "utils.hpp"
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <cctype>

namespace Utils {
    //==========================================================================
    // STRING HELPERS
    //==========================================================================
    
    std::vector<std::string> split(const std::string& str, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        
        for (char c : str) {
            if (c == delimiter) {
                if (!token.empty()) {
                    tokens.push_back(token);
                    token.clear();
                }
            } else {
                token += c;
            }
        }
        
        if (!token.empty()) {
            tokens.push_back(token);
        }
        
        return tokens;
    }
    
    std::string trim(const std::string& str) {
        size_t start = 0;
        size_t end = str.length();
        
        while (start < end && std::isspace(static_cast<unsigned char>(str[start]))) {
            start++;
        }
        
        while (end > start && std::isspace(static_cast<unsigned char>(str[end - 1]))) {
            end--;
        }
        
        return str.substr(start, end - start);
    }
    
    double parseJsonDouble(const std::string& json, const std::string& key) {
        // Find the key in the JSON string
        std::string searchKey = "\"" + key + "\"";
        size_t keyPos = json.find(searchKey);
        
        if (keyPos == std::string::npos) {
            return 0.0;
        }
        
        // Find the colon after the key
        size_t colonPos = json.find(':', keyPos);
        if (colonPos == std::string::npos) {
            return 0.0;
        }
        
        // Find the value after the colon
        size_t valueStart = colonPos + 1;
        while (valueStart < json.length() && 
               (std::isspace(static_cast<unsigned char>(json[valueStart])) || json[valueStart] == '"')) {
            valueStart++;
        }
        
        // Find the end of the value
        size_t valueEnd = valueStart;
        while (valueEnd < json.length() && 
               (std::isdigit(static_cast<unsigned char>(json[valueEnd])) || 
                json[valueEnd] == '.' || 
                json[valueEnd] == '-' || 
                json[valueEnd] == 'e' || 
                json[valueEnd] == 'E' ||
                json[valueEnd] == '+')) {
            valueEnd++;
        }
        
        if (valueEnd > valueStart) {
            std::string valueStr = json.substr(valueStart, valueEnd - valueStart);
            try {
                return std::stod(valueStr);
            } catch (...) {
                return 0.0;
            }
        }
        
        return 0.0;
    }
    
    //==========================================================================
    // FILE HELPERS
    //==========================================================================
    
    bool fileExists(const char* filename) {
        FILE* file = fopen(filename, "r");
        if (file) {
            fclose(file);
            return true;
        }
        return false;
    }
    
    std::string readFile(const char* filename) {
        FILE* file = fopen(filename, "r");
        if (!file) {
            return "";
        }
        
        // Get file size
        fseek(file, 0, SEEK_END);
        long size = ftell(file);
        fseek(file, 0, SEEK_SET);
        
        if (size <= 0) {
            fclose(file);
            return "";
        }
        
        // Read file content
        std::string content;
        content.resize(size);
        size_t bytesRead = fread(&content[0], 1, size, file);
        content.resize(bytesRead);
        
        fclose(file);
        return content;
    }
    
    bool writeFile(const char* filename, const std::string& content) {
        FILE* file = fopen(filename, "w");
        if (!file) {
            return false;
        }
        
        size_t written = fwrite(content.c_str(), 1, content.length(), file);
        fclose(file);
        
        return written == content.length();
    }
    
    bool appendFile(const char* filename, const std::string& content) {
        FILE* file = fopen(filename, "a");
        if (!file) {
            return false;
        }
        
        size_t written = fwrite(content.c_str(), 1, content.length(), file);
        fclose(file);
        
        return written == content.length();
    }
    
    //==========================================================================
    // CONTROLLER HELPERS
    //==========================================================================
    
    void waitForButton(pros::Controller& controller, pros::controller_digital_e_t button) {
        // Wait for button release first
        while (controller.get_digital(button)) {
            pros::delay(10);
        }
        
        // Wait for button press
        while (!controller.get_digital(button)) {
            pros::delay(10);
        }
        
        // Wait for button release
        while (controller.get_digital(button)) {
            pros::delay(10);
        }
    }
    
    void waitForAnyButton(pros::Controller& controller) {
        while (true) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_A) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ||
                controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                break;
            }
            pros::delay(10);
        }
        
        // Debounce
        pros::delay(100);
    }
    
    void rumble(pros::Controller& controller, const char* pattern) {
        controller.rumble(pattern);
    }
}

