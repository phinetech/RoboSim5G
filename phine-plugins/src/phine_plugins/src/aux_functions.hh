/* Copyright 2025 phine.tech GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in wrSiting, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/

#ifndef AUX_FUNCTIONS_HH
#define AUX_FUNCTIONS_HH

#include <algorithm>
#include <arpa/inet.h>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <regex>
#include <sdf/sdf.hh>
#include <set>
#include <sstream>
#include <string>
#include <vector>

// Enum for log levels
enum class LogLevel { DEBUG, WARN, ERR, INFO };

/**
 * @class RoboSimLogger
 * @brief Provides static logging utilities for RoboSim with support for log
 * levels and colored output.
 *
 * This class offers static methods to log messages at various log levels,
 * convert log levels to their string representations, and retrieve color codes
 * for enhanced terminal output.
 *
 * @method Log
 * @brief Logs a message with the specified log level.
 * @param level The log level to use for the message.
 * @param message The message to be logged.
 *
 * @method LogLevelToString
 * @brief Converts a LogLevel enum value to its corresponding string
 * representation.
 * @param level The log level to convert.
 * @return The string representation of the log level.
 *
 * @method GetColorCode
 * @brief Retrieves the color code associated with a specific log level.
 * @param level The log level for which to get the color code.
 * @return The color code string for the specified log level.
 */
class RoboSimLogger {
  public:
    // Static method to log messages with the given log level
    static void Log(LogLevel level, const std::string &message);

  private:
    // Helper method to convert enum LogLevel to string
    static std::string LogLevelToString(LogLevel level);

    // Helper method to get the color code for the log level
    static std::string GetColorCode(LogLevel level);
};

/**
 * @brief Modifies the value associated with a given key in a Docker
 * configuration file.
 *
 * This function searches for the specified key in the Docker configuration file
 * located at `file_path` and updates its value to `new_value`. Optionally,
 * debug information can be printed if `debug` is set to true.
 *
 * @param file_path The path to the Docker configuration file to be modified.
 * @param key The key in the configuration file whose value should be updated.
 * @param new_value The new value to assign to the specified key.
 * @param debug If true, enables debug output for tracing the modification
 * process.
 */
void modify_service_name(const std::string &file_path,
			 const std::string &old_key, const std::string &new_key,
			 bool debug);

/**
 * @brief Modifies the value associated with a specified key in a Docker
 * configuration file.
 *
 * This function searches for the given key in the specified file and updates
 * its value to the provided new_value. Optionally, debug information can be
 * printed during the operation.
 *
 * @param file_path The path to the Docker configuration file to be modified.
 * @param key The key whose value should be updated in the configuration file.
 * @param new_value The new value to assign to the specified key.
 * @param debug If true, enables debug output for tracing the modification
 * process.
 */
void modify_dockerC(const std::string &file_path, const std::string &key,
		    const std::string &new_value, bool debug);

/**
 * @brief Modifies the value associated with a specified key in a gNb
 * configuration file.
 *
 * This function searches for the given key in the specified gNb configuration
 * file and updates its value to the provided new_value. Optionally, debug
 * information can be printed during the operation.
 *
 * @param file_path The path to the gNb configuration file to be modified.
 * @param key The key whose value should be updated in the configuration file.
 * @param new_value The new value to assign to the specified key.
 * @param debug If true, enables debug output for tracing the modification
 * process.
 */
void modify_conf(const std::string &file_path, const std::string &key,
		 const std::string &new_value, bool debug);

// Function to retrieve parameters from SDF
/**
 * @brief Retrieves the value of a specified parameter from an SDF element.
 *
 * This function searches for a parameter with the given name within the
 * provided SDF element. If the parameter is found, its value is assigned to
 * paramValue.
 *
 * @param sdfClone      Pointer to the SDF element to search within.
 * @param paramName     Name of the parameter to retrieve.
 * @param paramValue    Reference to a string where the parameter value will be
 * stored if found.
 * @return true if the parameter was found and its value retrieved; false
 * otherwise.
 */
bool GetParamFromSDF(sdf::ElementPtr sdfClone, const std::string &paramName,
		     std::string &paramValue);

/**
 * @brief Converts a string representation to a boolean value.
 *
 * This function interprets the input string and returns its boolean equivalent.
 * Common true values are "true", "1", "yes", "on" (case-insensitive).
 * Common false values are "false", "0", "no", "off" (case-insensitive).
 * Behavior for unrecognized strings is implementation-defined.
 *
 * @param str The input string to convert.
 * @return true if the string represents a true value, false otherwise.
 */
bool string_to_bool(const std::string &str);

// function that takes the gNB name extraxt its number and converts it to
// hexadecimal to generate the gNB ID
/**
 * @brief Extracts a specific part of the input string and converts it to a
 * hexadecimal representation.
 *
 * This function processes the provided input string, extracts the relevant
 * portion (the extraction logic depends on the implementation), and returns its
 * hexadecimal representation as a std::string.
 *
 * @param input The input string to be processed and converted.
 * @return std::string The hexadecimal representation of the extracted part of
 * the input.
 */
std::string extractAndConvertToHex(const std::string &input);

/**
 * @brief Generates a list of IP addresses from a given subnet.
 *
 * This function takes a subnet in CIDR notation (e.g., "192.168.1.0/24")
 * and returns a vector containing all possible IP addresses within that subnet.
 *
 * @param subnet The subnet in CIDR notation as a string.
 * @return std::vector<std::string> A vector of IP addresses as strings.
 *
 * @note The function does not validate the subnet format. Invalid input may
 * result in undefined behavior.
 */
std::vector<std::string> generate_ips_from_subnet(const std::string &subnet);
/**
 * @brief Replaces the whitelist addresses for network interfaces within a
 * specified subnet.
 *
 * This function updates the whitelist addresses in the given file, ensuring
 * that only addresses within the provided subnet are included. Useful for
 * network configuration and access control.
 *
 * @param subnet The subnet (in CIDR notation) to filter whitelist addresses.
 * @param file_path The path to the file containing interface whitelist
 * addresses.
 * @param debug If true, enables debug output for troubleshooting.
 */
void replace_interface_whitelist_addresses(const std::string &subnet,
					   const std::string &file_path,
					   bool debug);

#endif // AUX_FUNCTIONS_HH