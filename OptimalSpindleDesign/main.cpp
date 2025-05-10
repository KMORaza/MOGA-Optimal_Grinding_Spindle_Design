#include "SpindleSimulation.hpp"
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>

void clearInputBuffer() {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

template<typename T>
T getNumericInput(const std::string& prompt, T min, T max) {
    T value;
    while (true) {
        std::cout << prompt;
        if (std::cin >> value && value >= min && value <= max) {
            clearInputBuffer();
            return value;
        }
        std::cout << "Invalid input. Must be between " << min << " and " << max << ".\n";
        clearInputBuffer();
    }
}

std::string getChoiceInput(const std::string& prompt, const std::vector<std::string>& options) {
    std::cout << prompt << "\n";
    for (size_t i = 0; i < options.size(); ++i) {
        std::cout << i + 1 << ". " << options[i] << "\n";
    }
    int choice = getNumericInput("Enter choice (1-" + std::to_string(options.size()) + "): ", 1, static_cast<int>(options.size()));
    return options[choice - 1];
}

SpindleParameters getParameters() {
    SpindleParameters params;

    std::vector<std::string> spindleTypes = {"Belt-Driven", "Direct-Drive", "Motorized"};
    params.setSpindleType(getChoiceInput("Select Spindle Type:", spindleTypes));

    params.setPowerRating(getNumericInput("Enter Power Rating (kW, 0.5-50): ", 0.5, 50.0));
    params.setMaxSpeed(getNumericInput("Enter Max Speed (RPM, 1000-30000): ", 1000, 30000));
    params.setWheelDiameter(getNumericInput("Enter Wheel Diameter (mm, 50-1000): ", 50.0, 1000.0));

    std::vector<std::string> bearingTypes = {"Angular Contact", "Hybrid Ceramic"};
    params.setBearingType(getChoiceInput("Select Bearing Type:", bearingTypes));

    params.setBearingPreload(getNumericInput("Enter Bearing Preload (N, 100-2000): ", 100.0, 2000.0));

    std::vector<std::string> coolingTypes = {"Liquid", "Air"};
    params.setCoolingType(getChoiceInput("Select Cooling Type:", coolingTypes));

    std::vector<std::string> lubricationTypes = {"Grease", "Oil-Mist", "Oil-Air"};
    params.setLubricationType(getChoiceInput("Select Lubrication Type:", lubricationTypes));

    std::vector<std::string> toolInterfaces = {"Precision Collet", "Hydraulic Chuck", "HSK"};
    params.setToolInterface(getChoiceInput("Select Tool Interface:", toolInterfaces));

    params.setAlignmentTolerance(getNumericInput("Enter Alignment Tolerance (mm, 0.0001-0.01): ", 0.0001, 0.01));

    return params;
}

void predictMaintenance(SpindleSimulation& sim, const SpindleParameters& params) {
    std::vector<double> loadProfile = sim.generateDynamicLoadProfile(params, 1.0, 1.0);
    double vibration = sim.estimateVibration(params);
    double temperature = sim.estimateTemperatureRise(params) + 20.0;
    double avgLoad = std::accumulate(loadProfile.begin(), loadProfile.end(), 0.0) / loadProfile.size();
    double bearingLife = sim.calculateBearingL10Life(params, loadProfile);
    double spindleLife = sim.calculateSpindleFatigueLife(params, loadProfile);
    double wheelWear = sim.calculateWheelWear(params, loadProfile, 1.0);
    double wearVibration = sim.calculateWearInducedVibration(params, wheelWear);
    double totalVibration = vibration + wearVibration;

    int maintenanceNeeded = sim.predictMaintenance(totalVibration, temperature, avgLoad, bearingLife, spindleLife, wheelWear);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "=== Maintenance Prediction ===\n\n";
    std::cout << "Vibration: " << totalVibration << " mm/s\n";
    std::cout << "Temperature: " << temperature << "°C\n";
    std::cout << "Average Load: " << avgLoad << " N\n";
    std::cout << "Bearing Life: " << bearingLife << " hours\n";
    std::cout << "Spindle Life: " << (spindleLife * 100) << "%\n";
    std::cout << "Wheel Wear: " << wheelWear << " mm\n";
    std::cout << (maintenanceNeeded == 1 ? "Maintenance Needed: Yes (e.g., bearing replacement, wheel dressing)\n" :
                                           "Maintenance Needed: No\n");
}

int main() {
    SpindleSimulation sim;
    std::cout << "Grinding Spindle Design Simulation\n";

    try {
        while (true) {
            std::cout << "\nSelect Operation:\n";
            std::cout << "1. Run Simulation\n";
            std::cout << "2. Run Time-Based Simulation\n";
            std::cout << "3. Generate Maintenance Schedule\n";
            std::cout << "4. Predict Maintenance\n";
            std::cout << "5. Optimize Spindle Arrangement\n";
            std::cout << "6. Exit\n";
            int choice = getNumericInput("Enter choice (1-6): ", 1, 6);

            if (choice == 6) break;

            try {
                if (choice == 1) {
                    SpindleParameters params = getParameters();
                    std::cout << sim.simulate(params) << "\n";
                } else if (choice == 2) {
                    SpindleParameters params = getParameters();
                    double duration = getNumericInput("Enter Simulation Duration (s, >0): ", 0.1, 1000.0);
                    std::cout << sim.simulateTimeBased(params, duration) << "\n";
                } else if (choice == 3) {
                    SpindleParameters params = getParameters();
                    std::cout << sim.generateMaintenanceSchedule(params) << "\n";
                } else if (choice == 4) {
                    SpindleParameters params = getParameters();
                    predictMaintenance(sim, params);
                } else if (choice == 5) {
                    double duration = getNumericInput("Enter Simulation Duration (s, >0): ", 0.1, 1000.0);
                    double loadFactor = getNumericInput("Enter Load Factor (0.5-2.0): ", 0.5, 2.0);
                    std::cout << sim.optimizeSpindleArrangement(duration, loadFactor, 50, 20) << "\n";
                }
            } catch (const std::exception& e) {
                std::cerr << "Error: " << e.what() << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
    } catch (...) {
        std::cerr << "Unknown fatal error occurred\n";
    }

    std::cout << "\nPress Enter to exit...\n";
    std::cin.get();
    return 0;
}