/* Copyright 2025 phine.tech GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.*/

#include "aux_functions.hh"

// Convert LogLevel enum to string representation
/**
 * @brief Converts a LogLevel enum value to its corresponding string
 * representation.
 *
 * @param level The LogLevel value to convert.
 * @return A string representing the log level ("DEBUG", "WARN", "ERR", "INFO",
 * or "UNKNOWN" for unrecognized values).
 */
std::string RoboSimLogger::LogLevelToString(LogLevel level) {
    switch (level) {
    case LogLevel::DEBUG:
	return "DEBUG";
    case LogLevel::WARN:
	return "WARN";
    case LogLevel::ERR:
	return "ERR";
    case LogLevel::INFO:
	return "INFO";
    default:
	return "UNKNOWN";
    }
}

// Get the color code for each log level
/**
 * Returns the ANSI color code string corresponding to the specified log level.
 *
 * @param level The log level for which to retrieve the color code. Supported
 * values are:
 *              - LogLevel::DEBUG: Blue
 *              - LogLevel::WARN: Yellow
 *              - LogLevel::ERR: Red
 *              - LogLevel::INFO: Green
 * @return A string containing the ANSI escape code for the color associated
 * with the given log level. Returns the default color code ("\033[0m") if the
 * log level is unrecognized.
 */
std::string RoboSimLogger::GetColorCode(LogLevel level) {
    switch (level) {
    case LogLevel::DEBUG:
	return "\033[34m"; // Blue
    case LogLevel::WARN:
	return "\033[33m"; // Yellow
    case LogLevel::ERR:
	return "\033[31m"; // Red
    case LogLevel::INFO:
	return "\033[32m"; // Green
    default:
	return "\033[0m"; // Default color
    }
}

// Log the message with the appropriate level
/**
 * @brief Logs a message with a specified log level, applying color formatting
 * to the output.
 *
 * Outputs the log message to the standard output stream, prefixing it with the
 * log level and applying a color code based on the log level for better
 * readability in the console.
 *
 * @param level The severity level of the log message (e.g., INFO, WARNING,
 * ERROR).
 * @param message The message string to be logged.
 */
void RoboSimLogger::Log(LogLevel level, const std::string &message) {
    std::string level_str = LogLevelToString(level);
    std::string color_code = GetColorCode(level);

    // Output the entire log message with the color code applied to the whole
    // line
    std::cout << color_code << "[RoboSim5g] [" << level_str << "] " << message
	      << "\033[0m" << std::endl;
}

/**
 * @brief Modifies the value associated with a specified key in a file.
 *
 * This function searches for a line in the file at `file_path` that contains
 * the given `key` followed by a colon and a value, and replaces the value with
 * `new_value`. The function preserves the original indentation of the matched
 * line. If `debug` is true, debug messages are logged.
 *
 * @param file_path The path to the file to be modified.
 * @param key The key whose value should be modified.
 * @param new_value The new value to assign to the key.
 * @param debug If true, enables debug logging.
 */
void modify_dockerC(const std::string &file_path, const std::string &key,
		    const std::string &new_value, bool debug) {
    // Open the file for reading
    std::ifstream file_in(file_path);
    if (!file_in) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "Error opening file for reading: " + file_path);
	return;
    }

    std::stringstream modified_content;
    std::string line;

    // Create a regex pattern to match the key followed by a colon and its value
    std::string regex_pattern = R"(\b)" + key + R"(\b\s*:\s*\S+)";

    // Iterate through each line of the file
    while (std::getline(file_in, line)) {
	std::regex pattern(regex_pattern);

	// Check if the line matches the pattern
	if (std::regex_search(line, pattern)) {
	    if (debug) {
		RoboSimLogger::Log(LogLevel::DEBUG, "Matched Line: " + line);
	    }

	    std::smatch match;
	    // Capture leading whitespace for proper formatting
	    if (std::regex_search(line, match, std::regex(R"(^\s*)"))) {
		std::string indentation = match.str(0);
		// Replace the key's value with the new value
		line = indentation + key + " : " + new_value;
	    }
	}

	// Append the (modified or unmodified) line to the output content
	modified_content << line << "\n";
    }

    file_in.close();

    // Open the file for writing and overwrite it with the modified content
    std::ofstream file_out(file_path, std::ios::trunc);
    if (!file_out) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "Error opening file for writing: " + file_path);
	return;
    }

    file_out << modified_content.str();
    file_out.close();

    if (debug) {
	RoboSimLogger::Log(LogLevel::DEBUG, "File modified successfully.");
    }
}

/**
 * @brief Modifies the value of a specified key in a configuration file.
 *
 * This function searches for a line in the given file that matches the
 * specified key (in the form "key = value") and replaces its value with the
 * provided new value. The function preserves all other lines and writes the
 * modified content back to the file.
 *
 * @param file_path The path to the configuration file to be modified.
 * @param key The key whose value should be updated.
 * @param new_value The new value to assign to the key.
 * @param debug If true, debug messages will be logged during the operation.
 *
 * @note If the file cannot be opened for reading or writing, an error is logged
 * and the function returns.
 * @note The function overwrites the original file with the modified content.
 */
void modify_conf(const std::string &file_path, const std::string &key,
		 const std::string &new_value, bool debug) {
    // Open the file for reading
    std::ifstream file_in(file_path);
    if (!file_in) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "Error opening file for reading: " + file_path);
	return;
    }

    std::stringstream modified_content;
    std::string line;

    // Create a regex pattern to match the key followed by an equals sign and
    // its value
    std::string regex_pattern = R"(\b)" + key + R"(\b\s*=\s*\S+)";

    // Iterate through each line of the file
    while (std::getline(file_in, line)) {
	std::regex pattern(regex_pattern);

	// Check if the line matches the pattern
	if (std::regex_search(line, pattern)) {
	    if (debug) {
		RoboSimLogger::Log(LogLevel::DEBUG, "Matched Line: " + line);
	    }

	    std::smatch match;
	    std::string before_value;

	    // Capture the part of the line before the equals sign
	    if (std::regex_search(line, match, std::regex(R"((.*)\s*=\s*)"))) {
		before_value = match.str(1);
		// Replace the key's value with the new value, enclosing it in
		// quotes
		line = before_value + " = " + new_value;
	    }
	}

	// Append the (modified or unmodified) line to the output content
	modified_content << line << "\n";
    }

    file_in.close();

    // Open the file for writing and overwrite it with the modified content
    std::ofstream file_out(file_path, std::ios::trunc);
    if (!file_out) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "Error opening file for writing: " + file_path);
	return;
    }

    file_out << modified_content.str();
    file_out.close();

    if (debug) {
	RoboSimLogger::Log(LogLevel::DEBUG, "File modified successfully.");
    }
}

// Function to modify the service name in docker compose file (key : -> new_key
// :)
/**
 * @brief Modifies a specific key in a file, replacing it with a new key while
 * preserving its value.
 *
 * This function searches for lines in the specified file that contain the given
 * `old_key` followed by a colon, and replaces the key with `new_key`, keeping
 * any associated value intact. The function preserves indentation and only
 * updates lines where the key matches exactly. The modified content is written
 * back to the original file.
 *
 * @param file_path The path to the file to be modified.
 * @param old_key The key to search for and replace.
 * @param new_key The new key to use as a replacement.
 * @param debug If true, debug messages will be logged during processing.
 *
 * @note If the file cannot be opened for reading or writing, an error is logged
 * and the function returns early.
 */
void modify_service_name(const std::string &file_path,
			 const std::string &old_key, const std::string &new_key,
			 bool debug) {
    std::ifstream file_in(file_path);
    if (!file_in) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "Error opening file for reading: " + file_path);
	return;
    }

    std::stringstream modified_content;
    std::string line;

    // Create a regex pattern to find the old_key followed by a colon and
    // optional spaces or value
    std::string regex_pattern =
	R"(\b)" + old_key +
	R"(\b\s*:\s*)"; // Matches `old_key:` with optional spaces after it

    // Iterate over each line of the file
    while (std::getline(file_in, line)) {
	std::regex pattern(regex_pattern);

	// If the line contains the old_key: (with or without value)
	if (std::regex_search(line, pattern)) {
	    if (debug) {
		RoboSimLogger::Log(LogLevel::DEBUG, "Matched Line: " + line);
	    }

	    // Search for the key in the line and ensure it matches the old_key
	    std::smatch match;
	    if (std::regex_search(line, match, std::regex(R"(^\s*)"))) {
		std::string indentation =
		    match.str(0); // Capture leading whitespace

		// If there is a value after the colon, retain the value.
		// Otherwise, keep it empty.
		size_t pos = line.find(":");
		std::string value = "";
		if (pos != std::string::npos && pos + 1 < line.size()) {
		    value = line.substr(pos + 1); // Extract value if present
		}

		// If there's no value, ensure we just have `new_key:`
		line = indentation + new_key + " :" +
		       value; // Update the key while preserving any value
	    }
	}

	// Append the line (modified or not) to the output string
	modified_content << line << "\n";
    }

    file_in.close();

    // Open the file in write mode to overwrite it with the modified content
    std::ofstream file_out(file_path, std::ios::trunc);
    if (!file_out) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "Error opening file for writing: " + file_path);
	return;
    }

    // Write the modified content back to the file
    file_out << modified_content.str();
    file_out.close();

    if (debug) {
	RoboSimLogger::Log(LogLevel::DEBUG, "File modified successfully.");
    }
}

// Function to retrieve parameters from SDF
/**
 * @brief Retrieves a parameter value from an SDF element.
 *
 * This function attempts to extract the value of a specified parameter from a
 * given SDF (Simulation Description Format) element. If the parameter is found
 * and its value is not empty, the value is assigned to the output parameter and
 * the function returns true. Otherwise, an error is logged and the function
 * returns false.
 *
 * @param sdfClone      Pointer to the SDF element to search for the parameter.
 * @param paramName     Name of the parameter to retrieve.
 * @param paramValue    Reference to a string where the parameter value will be
 * stored.
 * @return true if the parameter was found and is not empty, false otherwise.
 */
bool GetParamFromSDF(sdf::ElementPtr sdfClone, const std::string &paramName,
		     std::string &paramValue) {
    auto sdfElem = sdfClone->GetElement(paramName);
    if (sdfElem) {
	paramValue = sdfElem->Get<std::string>();
    }

    if (paramValue.empty()) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "gNb_plugin found an empty " + paramName +
			       " parameter. Failed to initialize.");
	return false;
    }

    RoboSimLogger::Log(LogLevel::INFO, paramName + " is: " + paramValue);
    return true;
}

/**
 * @brief Converts a string representation of a boolean value to a bool.
 *
 * Accepts case-insensitive string values: "true", "1", "yes" (returns true),
 * and "false", "0", "no" (returns false). Throws std::invalid_argument for any
 * other input.
 *
 * @param str The input string to convert.
 * @return bool The corresponding boolean value.
 * @throws std::invalid_argument If the input string does not represent a valid
 * boolean.
 */
bool string_to_bool(const std::string &str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
		   ::tolower);

    if (lower_str == "true" || lower_str == "1" || lower_str == "yes") {
	return true;
    } else if (lower_str == "false" || lower_str == "0" || lower_str == "no") {
	return false;
    }

    throw std::invalid_argument("Invalid string for boolean conversion");
}

/**
 * @brief Extracts the trailing decimal number from the end of the input string
 * and converts it to an uppercase hexadecimal string.
 *
 * This function scans the input string from the end, collects consecutive
 * digits to form a number, converts that number to an integer, and then formats
 * it as a hexadecimal string prefixed with "0x". If no trailing number is
 * found, it returns a message indicating so.
 *
 * @param input The input string potentially ending with a decimal number.
 * @return std::string The hexadecimal representation of the extracted number
 * (e.g., "0x1A3"), or "No number found at the end of the string" if no trailing
 * number exists.
 */
std::string extractAndConvertToHex(const std::string &input) {
    // Start from the end of the string to find the number
    std::string number_str = "";

    // Iterate backwards through the string to find the number
    for (int i = input.size() - 1; i >= 0; --i) {
	if (isdigit(input[i])) {
	    number_str =
		input[i] + number_str; // Add the digit to the number string
	} else {
	    break; // Stop when a non-digit is found
	}
    }

    // If we found a number, convert it to hexadecimal
    if (!number_str.empty()) {
	int number =
	    std::stoi(number_str); // Convert the string number to an integer

	// Use stringstream to format the number in hexadecimal
	std::stringstream hex_stream;
	hex_stream << "0x" << std::uppercase << std::hex
		   << number; // Convert to hex and make it uppercase

	return hex_stream.str(); // Return the hexadecimal string
    } else {
	return "No number found at the end of the string";
    }
}

/**
 * @brief Generates a list of IP addresses within a given IPv4 subnet (CIDR
 * notation).
 *
 * This function parses the provided subnet string (e.g., "192.168.1.0/24"),
 * calculates the range of valid host IP addresses (excluding network and
 * broadcast addresses), and returns them as a vector of strings.
 *
 * @param subnet The subnet in CIDR notation (e.g., "192.168.1.0/24").
 * @return std::vector<std::string> List of IP addresses within the subnet.
 *
 * @note Returns an empty vector if the input is invalid or if no usable IPs
 * exist.
 */
std::vector<std::string> generate_ips_from_subnet(const std::string &subnet) {
    std::vector<std::string> result;
    size_t slash_pos = subnet.find('/');
    if (slash_pos == std::string::npos)
	return result;

    std::string base_ip = subnet.substr(0, slash_pos);
    int prefix = std::stoi(subnet.substr(slash_pos + 1));

    in_addr addr;
    if (inet_aton(base_ip.c_str(), &addr) == 0)
	return result;

    uint32_t ip = ntohl(addr.s_addr);
    uint32_t mask = (0xFFFFFFFF << (32 - prefix)) & 0xFFFFFFFF;
    uint32_t start = (ip & mask) + 1; // skip network address
    uint32_t end = (ip | ~mask) - 1;  // skip broadcast address

    // Loop through all usable host IPs in the subnet
    for (uint32_t i = start; i <= end; ++i) {
	in_addr current;
	current.s_addr = htonl(i);
	result.push_back(inet_ntoa(current)); // convert to string
    }

    return result;
}

/**
 * @brief Replaces the <interfaceWhiteList> block in an XML file with addresses
 * generated from a given subnet.
 *
 * This function reads the specified file, generates a list of IP addresses from
 * the provided subnet, and replaces the contents of the <interfaceWhiteList>
 * XML block with these addresses, ensuring that "127.0.0.1" is always included.
 * The updated content is then written back to the file.
 *
 * @param subnet The subnet string (e.g., "192.168.1.0/24") from which IP
 * addresses are generated.
 * @param file_path The path to the XML file to be modified.
 * @param debug Optional flag to enable debug logging (default: false).
 *
 * @note Logs errors if the file cannot be opened, the subnet is invalid, or the
 * file cannot be written.
 * @note Uses regex to match and replace the <interfaceWhiteList> block in a
 * multiline-safe manner.
 */
void replace_interface_whitelist_addresses(const std::string &subnet,
					   const std::string &file_path,
					   bool debug = false) {
    (void)debug; // Avoid unused parameter warning

    std::ifstream file_in(file_path);
    if (!file_in) {
	RoboSimLogger::Log(LogLevel::ERR, "Failed to open file: " + file_path);
	return;
    }

    std::stringstream buffer;
    buffer << file_in.rdbuf();
    std::string content = buffer.str();
    file_in.close();

    std::vector<std::string> ips = generate_ips_from_subnet(subnet);
    if (ips.empty()) {
	RoboSimLogger::Log(LogLevel::ERR, "Invalid or empty subnet: " + subnet);
	return;
    }

    std::set<std::string> unique_ips(ips.begin(), ips.end());
    unique_ips.insert("127.0.0.1"); // ensure loopback

    std::stringstream new_block;
    for (const std::string &ip : unique_ips) {
	new_block << "                <address>" << ip << "</address>\n";
    }

    // ✅ Use regex that matches multiline blocks
    std::regex whitelist_block(
	R"(<interfaceWhiteList>[\s\S]*?</interfaceWhiteList>)");
    content = std::regex_replace(content, whitelist_block,
				 "<interfaceWhiteList>\n" + new_block.str() +
				     "            </interfaceWhiteList>");

    std::ofstream file_out(file_path, std::ios::trunc);
    if (!file_out) {
	RoboSimLogger::Log(LogLevel::ERR,
			   "Failed to write to file: " + file_path);
	return;
    }

    file_out << content;
    file_out.close();

    RoboSimLogger::Log(
	LogLevel::DEBUG,
	"Successfully updated <interfaceWhiteList> with subnet: " + subnet);
}
