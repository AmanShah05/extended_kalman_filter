#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_set>
#include <string>

// Define the required columns
std::unordered_set<std::string> get_required_columns() {
    std::unordered_set<std::string> columns;
    columns.insert("gps/hdt_valid");
    columns.insert("gps/vel_valid");
    columns.insert("gps/connected");
    columns.insert("gps/hdt/heading");
    columns.insert("gps/hdt/pitch");
    columns.insert("gps/vel/velocity/0");
    columns.insert("gps/vel/velocity/1");
    columns.insert("gps/vel/velocity/2");
    columns.insert("gps/hdt/headingAccuracy");
    columns.insert("gps/hdt/pitchAccuracy");
    columns.insert("gps/vel/velocityAcc/0");
    columns.insert("gps/vel/velocityAcc/1");
    columns.insert("gps/vel/velocityAcc/2");
    columns.insert("sensors/altimeter/altitude_filt");
    columns.insert("sensors/optic_downward/detla_x_flow");
    columns.insert("sensors/optic_downward/detla_y_flow");
    columns.insert("sensors/optic_downward/confidence");
    columns.insert("sensors/optic_forward/detla_x_flow");
    columns.insert("sensors/optic_forward/detla_y_flow");
    columns.insert("sensors/optic_forward/confidence");
    columns.insert("ins/raw/lin/acc/0");
    columns.insert("ins/raw/lin/acc/1");
    columns.insert("ins/raw/lin/acc/2");
    columns.insert("ins/raw/ang/vel/0");
    columns.insert("ins/raw/ang/vel/1");
    columns.insert("ins/raw/ang/vel/2");
    columns.insert("sensors/lidar_vel_filt");
    columns.insert("sensors/lidar_dist_filt");
    columns.insert("sensors/gh_dist");
    columns.insert("sensors/gh_vel_filt");
    columns.insert("state/curr");
    return columns;
}

// Define the columns to compare
std::unordered_set<std::string> get_comparison_columns() {
    std::unordered_set<std::string> columns;
    columns.insert("gps/vel/velocity/0");
    columns.insert("gps/vel/velocity/1");
    columns.insert("gps/vel/velocity/2");
    return columns;
}

int main() {
    std::unordered_set<std::string> required_columns = get_required_columns();
    std::unordered_set<std::string> comparison_columns = get_comparison_columns();

    std::ifstream input_file("/Users/amanshah/Documents/Software Projects/Extended-Kalman-Filter/D0033-NIPAWIN-FS0003-R0001-telemetry.csv");
    std::ofstream output_file("filtered_data.csv");

    if (!input_file.is_open()) {
        std::cerr << "Error: Could not open input file." << std::endl;
        return 1;
    }
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open output file." << std::endl;
        return 1;
    }

    std::string line;
    std::vector<int> column_indices;
    std::vector<int> comparison_indices;
    std::vector<std::string> headers;

    // Read the header line
    if (std::getline(input_file, line)) {
        std::istringstream header_stream(line);
        std::string column;
        int index = 0;
        while (std::getline(header_stream, column, ',')) {
            if (required_columns.find(column) != required_columns.end()) {
                column_indices.push_back(index);
                headers.push_back(column);
            }
            if (comparison_columns.find(column) != comparison_columns.end()) {
                comparison_indices.push_back(index);
            }
            ++index;
        }
    }

    // Write the filtered headers to the output file, plus a column for the comparison result
    for (size_t i = 0; i < headers.size(); ++i) {
        output_file << headers[i];
        if (i < headers.size() - 1) {
            output_file << ",";
        }
    }
    output_file << ",new_data\n";

    std::vector<std::string> prev_comparison_values;
    bool is_first_row = true;

    // Read and filter the data rows
    while (std::getline(input_file, line)) {
        std::istringstream line_stream(line);
        std::string cell;
        std::vector<std::string> row;
        std::vector<std::string> comparison_values;
        int index = 0;
        while (std::getline(line_stream, cell, ',')) {
            if (std::find(column_indices.begin(), column_indices.end(), index) != column_indices.end()) {
                row.push_back(cell);
            }
            if (std::find(comparison_indices.begin(), comparison_indices.end(), index) != comparison_indices.end()) {
                comparison_values.push_back(cell);
            }
            ++index;
        }

        // Write the filtered row to the output file
        for (size_t i = 0; i < row.size(); ++i) {
            output_file << row[i];
            if (i < row.size() - 1) {
                output_file << ",";
            }
        }

        // Compare with the previous row's comparison columns
        if (is_first_row) {
            output_file << ",TRUE\n"; // First row has no previous row to compare to, so we output TRUE
            is_first_row = false;
        } else {
            bool is_same = (comparison_values == prev_comparison_values);
            output_file << "," << (is_same ? "FALSE" : "TRUE") << "\n";
        }

        // Update previous comparison values
        prev_comparison_values = comparison_values;
    }

    input_file.close();
    output_file.close();

    std::cout << "Filtered data has been saved to 'filtered_data.csv'" << std::endl;

    return 0;
}
