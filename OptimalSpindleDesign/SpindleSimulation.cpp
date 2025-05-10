#define _USE_MATH_DEFINES
#include "SpindleSimulation.hpp"
#include <sstream>
#include <algorithm>
#include <cmath>
#include <random>
#include <iomanip>
#include <limits>
#include <iostream>
#include <fstream>

std::vector<SpindleSimulation::DataPoint> SpindleSimulation::historicalData;

SpindleSimulation::SpindleSimulation() : rng(std::random_device{}()) {}

std::string SpindleSimulation::validateParameters(const SpindleParameters& params) const {
    if (params.getPowerRating() < 0.5 || params.getPowerRating() > 50.0)
        return "Error: Power rating must be between 0.5 and 50 kW\n";
    if (params.getMaxSpeed() < 1000 || params.getMaxSpeed() > 30000)
        return "Error: Max speed must be between 1000 and 30000 RPM\n";
    if (params.getWheelDiameter() < 50 || params.getWheelDiameter() > 1000)
        return "Error: Wheel diameter must be between 50 and 1000 mm\n";
    if (params.getBearingPreload() < 100 || params.getBearingPreload() > 2000)
        return "Error: Bearing preload must be between 100 and 2000 N\n";
    if (params.getAlignmentTolerance() < 0.0001 || params.getAlignmentTolerance() > 0.01)
        return "Error: Alignment tolerance must be between 0.0001 and 0.01 mm\n";
    return "Valid";
}

std::string SpindleSimulation::simulate(const SpindleParameters& params) {
    std::string validationResult = validateParameters(params);
    if (validationResult != "Valid")
        return validationResult;

    std::vector<SimulationScenario> scenarios = {
        {"High-Speed", 1.0, 0.8, 10.0},
        {"High-Torque", 0.6, 1.2, 10.0},
        {"Balanced", 0.8, 1.0, 10.0}
    };

    std::stringstream report;
    report << "=== Systematic Spindle Simulation Results ===\n\n";
    for (const auto& scenario : scenarios) {
        report << runSimulationStage(params, scenario);
    }
    report << generateComprehensiveReport(params, scenarios);
    return report.str();
}

std::string SpindleSimulation::runSimulationStage(const SpindleParameters& params, const SimulationScenario& scenario) {
    std::stringstream results;
    results << std::fixed << std::setprecision(2);
    results << "=== Scenario: " << scenario.name << " ===\n\n";

    SpindleParameters adjustedParams;
    adjustedParams.setSpindleType(params.getSpindleType());
    adjustedParams.setPowerRating(params.getPowerRating());
    adjustedParams.setMaxSpeed(static_cast<int>(params.getMaxSpeed() * scenario.speedFactor));
    adjustedParams.setWheelDiameter(params.getWheelDiameter());
    adjustedParams.setBearingType(params.getBearingType());
    adjustedParams.setBearingPreload(params.getBearingPreload());
    adjustedParams.setCoolingType(params.getCoolingType());
    adjustedParams.setLubricationType(params.getLubricationType());
    adjustedParams.setToolInterface(params.getToolInterface());
    adjustedParams.setAlignmentTolerance(params.getAlignmentTolerance());

    double requiredPower = calculateRequiredPower(adjustedParams.getWheelDiameter(), adjustedParams.getMaxSpeed());
    results << "Power Analysis: " << requiredPower << " kW required, " << adjustedParams.getPowerRating() << " kW provided\n";
    results << (requiredPower <= adjustedParams.getPowerRating() ? "Power rating sufficient\n" : "Warning: Power rating may be insufficient\n");

    results << "\nBearing Analysis:\n" << evaluateBearingPerformance(adjustedParams) << "\n";
    results << "Bearing Preload: " << adjustedParams.getBearingPreload() << " N\n";
    results << (adjustedParams.getBearingPreload() >= 300 && adjustedParams.getBearingPreload() <= 1000 ?
                "Preload within optimal range\n" : "Warning: Preload may cause excessive heat or play\n");

    results << "\nThermal Analysis:\n";
    double tempRise = estimateTemperatureRise(adjustedParams);
    double thermalExpansion = calculateThermalExpansion(tempRise);
    results << "Estimated temperature rise: " << tempRise << "°C\n";
    results << "Thermal expansion: " << std::setprecision(4) << thermalExpansion << " mm\n";
    results << (tempRise <= 30 ? "Thermal performance acceptable\n" : "Warning: Potential thermal issues\n");

    results << "\nVibration Analysis:\n";
    double vibrationLevel = estimateVibration(adjustedParams);
    double resonanceFreq = calculateResonanceFrequency(adjustedParams);
    results << "Estimated vibration level: " << vibrationLevel << " mm/s\n";
    results << "Resonance frequency: " << resonanceFreq << " Hz\n";
    results << (vibrationLevel <= 1.0 ? "Vibration within ISO 1940 G1 standards\n" : "Warning: Excessive vibration predicted\n");

    results << "\nAlignment Analysis:\n";
    results << "Alignment tolerance: " << std::setprecision(4) << adjustedParams.getAlignmentTolerance() << " mm\n";
    results << (adjustedParams.getAlignmentTolerance() <= 0.002 ? "Alignment within specifications\n" : "Warning: Alignment may cause chatter marks\n");

    results << "\nTool Interface Analysis:\n";
    results << "Tool Interface: " << adjustedParams.getToolInterface() << "\n";
    results << (adjustedParams.getToolInterface() == "HSK" && adjustedParams.getMaxSpeed() > 10000 ?
                "HSK interface optimal for high-speed operation\n" : "Tool interface suitable for specified parameters\n");

    results << "\nDynamic Load Profile:\n";
    std::vector<double> loadProfile = generateDynamicLoadProfile(adjustedParams, scenario.duration, scenario.loadFactor);
    results << "Dynamic Load (N) over " << scenario.duration << " seconds:\n";
    for (size_t i = 0; i < loadProfile.size(); ++i) {
        results << "t=" << (i * 0.1) << " s: " << loadProfile[i] << " N\n";
    }

    results << "\nFatigue Analysis:\n";
    double bearingLifeHours = calculateBearingL10Life(adjustedParams, loadProfile);
    results << "Bearing L10 Life: " << bearingLifeHours << " hours\n";
    results << (bearingLifeHours >= 20000 ? "Bearing life acceptable\n" : "Warning: Short bearing life predicted\n");

    double spindleLifePercentage = calculateSpindleFatigueLife(adjustedParams, loadProfile);
    results << "Spindle Shaft Remaining Life: " << (spindleLifePercentage * 100) << "%\n";
    results << (spindleLifePercentage >= 0.5 ? "Spindle shaft life acceptable\n" : "Warning: Spindle shaft may fail prematurely\n");

    results << "\nGrinding Wheel Wear Analysis:\n";
    double initialDiameter = adjustedParams.getWheelDiameter();
    double wear = calculateWheelWear(adjustedParams, loadProfile, scenario.duration);
    double remainingDiameter = initialDiameter - wear;
    double wearVibration = calculateWearInducedVibration(adjustedParams, wear);
    results << "Initial Wheel Diameter: " << initialDiameter << " mm\n";
    results << "Remaining Wheel Diameter: " << remainingDiameter << " mm\n";
    results << "Wear-Induced Vibration: " << wearVibration << " mm/s\n";
    results << (remainingDiameter >= initialDiameter * 0.8 ? "Wheel condition acceptable\n" : "Warning: Excessive wheel wear detected\n");
    results << (wearVibration <= 0.5 ? "Wear-induced vibration within limits\n" : "Warning: Increased vibration due to wheel imbalance\n");

    results << "\nMaintenance Prediction:\n";
    if (historicalData.empty()) generateHistoricalData();
    double totalVibration = vibrationLevel + wearVibration;
    double avgLoad = std::accumulate(loadProfile.begin(), loadProfile.end(), 0.0) / loadProfile.size();
    int maintenanceNeeded = predictMaintenance(totalVibration, tempRise + 20.0, avgLoad, bearingLifeHours, spindleLifePercentage, wear);
    results << (maintenanceNeeded == 1 ? "Maintenance Needed: Yes (e.g., bearing replacement, wheel dressing)\n" : "Maintenance Needed: No\n");

    int label = (totalVibration > 1.0 || bearingLifeHours < 5000 || spindleLifePercentage < 0.5 || wear > initialDiameter * 0.2) ? 1 : 0;
    historicalData.emplace_back(totalVibration, tempRise + 20.0, avgLoad, bearingLifeHours, spindleLifePercentage, wear, label);

    results << "\n";
    return results.str();
}

std::string SpindleSimulation::generateComprehensiveReport(const SpindleParameters& params, const std::vector<SimulationScenario>& scenarios) {
    std::stringstream report;
    report << std::fixed << std::setprecision(2);
    report << "=== Comprehensive Analysis ===\n\n";
    report << "Spindle Type: " << params.getSpindleType() << "\n";
    report << evaluateSpindleType(params) << "\n\n";

    report << "Summary Across Scenarios:\n";
    for (const auto& scenario : scenarios) {
        SpindleParameters adjustedParams;
        adjustedParams.setSpindleType(params.getSpindleType());
        adjustedParams.setPowerRating(params.getPowerRating());
        adjustedParams.setMaxSpeed(static_cast<int>(params.getMaxSpeed() * scenario.speedFactor));
        adjustedParams.setWheelDiameter(params.getWheelDiameter());
        adjustedParams.setBearingType(params.getBearingType());
        adjustedParams.setBearingPreload(params.getBearingPreload());
        adjustedParams.setCoolingType(params.getCoolingType());
        adjustedParams.setLubricationType(params.getLubricationType());
        adjustedParams.setToolInterface(params.getToolInterface());
        adjustedParams.setAlignmentTolerance(params.getAlignmentTolerance());

        std::vector<double> loadProfile = generateDynamicLoadProfile(adjustedParams, scenario.duration, scenario.loadFactor);
        double vibration = estimateVibration(adjustedParams);
        double tempRise = estimateTemperatureRise(adjustedParams);
        double bearingLife = calculateBearingL10Life(adjustedParams, loadProfile);
        double spindleLife = calculateSpindleFatigueLife(adjustedParams, loadProfile);
        double wheelWear = calculateWheelWear(adjustedParams, loadProfile, scenario.duration);
        double wearVibration = calculateWearInducedVibration(adjustedParams, wheelWear);

        report << "Scenario: " << scenario.name << "\n";
        report << " - Vibration: " << (vibration + wearVibration) << " mm/s\n";
        report << " - Temperature Rise: " << tempRise << "°C\n";
        report << " - Bearing Life: " << bearingLife << " hours\n";
        report << " - Spindle Life: " << (spindleLife * 100) << "%\n";
        report << " - Wheel Wear: " << wheelWear << " mm\n\n";
    }

    report << "Recommendations:\n";
    bool highVibration = false, highTemp = false, lowBearingLife = false;
    for (const auto& scenario : scenarios) {
        SpindleParameters adjustedParams;
        adjustedParams.setSpindleType(params.getSpindleType());
        adjustedParams.setPowerRating(params.getPowerRating());
        adjustedParams.setMaxSpeed(static_cast<int>(params.getMaxSpeed() * scenario.speedFactor));
        adjustedParams.setWheelDiameter(params.getWheelDiameter());
        adjustedParams.setBearingType(params.getBearingType());
        adjustedParams.setBearingPreload(params.getBearingPreload());
        adjustedParams.setCoolingType(params.getCoolingType());
        adjustedParams.setLubricationType(params.getLubricationType());
        adjustedParams.setToolInterface(params.getToolInterface());
        adjustedParams.setAlignmentTolerance(params.getAlignmentTolerance());

        std::vector<double> loadProfile = generateDynamicLoadProfile(adjustedParams, scenario.duration, scenario.loadFactor);
        double vibration = estimateVibration(adjustedParams);
        double tempRise = estimateTemperatureRise(adjustedParams);
        double bearingLife = calculateBearingL10Life(adjustedParams, loadProfile);
        double wheelWear = calculateWheelWear(adjustedParams, loadProfile, scenario.duration);
        double wearVibration = calculateWearInducedVibration(adjustedParams, wheelWear);

        if (vibration + wearVibration > 1.0) highVibration = true;
        if (tempRise > 30) highTemp = true;
        if (bearingLife < 20000) lowBearingLife = true;
    }

    if (highVibration)
        report << " - Consider upgrading to Hybrid Ceramic bearings or HSK tool interface to reduce vibration.\n";
    if (highTemp)
        report << " - Switch to Liquid cooling to improve thermal performance.\n";
    if (lowBearingLife)
        report << " - Optimize lubrication type (e.g., Oil-Air) or reduce bearing preload to extend bearing life.\n";
    if (!highVibration && !highTemp && !lowBearingLife)
        report << " - Current configuration is robust across tested scenarios.\n";

    return report.str();
}

std::string SpindleSimulation::simulateTimeBased(const SpindleParameters& params, double duration) {
    std::string validationResult = validateParameters(params);
    if (validationResult != "Valid")
        return validationResult;

    std::stringstream results;
    results << std::fixed << std::setprecision(2);
    results << "=== Time-Based Spindle Simulation (Duration: " << duration << " s) ===\n\n";

    double timeStep = 0.1;
    int steps = static_cast<int>(duration / timeStep);
    std::vector<double> vibrationHistory, temperatureHistory;
    std::vector<double> loadProfile = generateDynamicLoadProfile(params, duration, 1.0);

    double currentTemp = 20.0;
    for (int i = 0; i < steps; ++i) {
        double load = loadProfile[i];
        double vibration = estimateVibration(params, load);
        currentTemp += estimateTemperatureRise(params, load) * timeStep / 10.0;
        vibrationHistory.push_back(vibration);
        temperatureHistory.push_back(currentTemp);

        if (i % 10 == 0) {
            results << "t=" << (i * timeStep) << " s: Vibration=" << vibration << " mm/s, Temperature=" << currentTemp << "°C, Load=" << load << " N\n";
        }
    }

    double avgVibration = std::accumulate(vibrationHistory.begin(), vibrationHistory.end(), 0.0) / vibrationHistory.size();
    double maxVibration = *std::max_element(vibrationHistory.begin(), vibrationHistory.end());
    double avgTemp = std::accumulate(temperatureHistory.begin(), temperatureHistory.end(), 0.0) / temperatureHistory.size();
    double maxTemp = *std::max_element(temperatureHistory.begin(), temperatureHistory.end());

    results << "\nSummary:\n";
    results << "Average Vibration: " << avgVibration << " mm/s\n";
    results << "Maximum Vibration: " << maxVibration << " mm/s\n";
    results << "Average Temperature: " << avgTemp << "°C\n";
    results << "Maximum Temperature: " << maxTemp << "°C\n";

    results << "\nFatigue Analysis:\n";
    double bearingLifeHours = calculateBearingL10Life(params, loadProfile);
    results << "Bearing L10 Life: " << bearingLifeHours << " hours\n";
    results << (bearingLifeHours >= 20000 ? "Bearing life acceptable\n" : "Warning: Short bearing life predicted\n");

    double spindleLifePercentage = calculateSpindleFatigueLife(params, loadProfile);
    results << "Spindle Shaft Remaining Life: " << (spindleLifePercentage * 100) << "%\n";
    results << (spindleLifePercentage >= 0.5 ? "Spindle shaft life acceptable\n" : "Warning: Spindle shaft may fail prematurely\n");

    results << "\nGrinding Wheel Wear Analysis:\n";
    double initialDiameter = params.getWheelDiameter();
    double wear = calculateWheelWear(params, loadProfile, duration);
    double remainingDiameter = initialDiameter - wear;
    double wearVibration = calculateWearInducedVibration(params, wear);
    results << "Initial Wheel Diameter: " << initialDiameter << " mm\n";
    results << "Remaining Wheel Diameter: " << remainingDiameter << " mm\n";
    results << "Wear-Induced Vibration: " << wearVibration << " mm/s\n";
    results << (remainingDiameter >= initialDiameter * 0.8 ? "Wheel condition acceptable\n" : "Warning: Excessive wheel wear detected\n");
    results << (wearVibration <= 0.5 ? "Wear-induced vibration within limits\n" : "Warning: Increased vibration due to wheel imbalance\n");

    results << "\nMaintenance Prediction:\n";
    if (historicalData.empty()) generateHistoricalData();
    double totalVibration = maxVibration + wearVibration;
    double avgLoad = std::accumulate(loadProfile.begin(), loadProfile.end(), 0.0) / loadProfile.size();
    int maintenanceNeeded = predictMaintenance(totalVibration, maxTemp, avgLoad, bearingLifeHours, spindleLifePercentage, wear);
    results << (maintenanceNeeded == 1 ? "Maintenance Needed: Yes (e.g., bearing replacement, wheel dressing)\n" : "Maintenance Needed: No\n");

    int label = (totalVibration > 1.0 || bearingLifeHours < 5000 || spindleLifePercentage < 0.5 || wear > initialDiameter * 0.2) ? 1 : 0;
    historicalData.emplace_back(totalVibration, maxTemp, avgLoad, bearingLifeHours, spindleLifePercentage, wear, label);

    return results.str();
}

std::string SpindleSimulation::generateMaintenanceSchedule(const SpindleParameters& params) {
    std::stringstream schedule;
    schedule << "=== Spindle Maintenance Schedule ===\n\n";

    int bearingInterval = params.getBearingType() == "Hybrid Ceramic" ? 2000 : 1500;
    schedule << "Bearing Inspection: Every " << bearingInterval << " operating hours\n";
    schedule << " - Check for wear, preload, and runout\n";
    schedule << " - Verify ABEC 7 precision standards\n\n";

    schedule << "Lubrication Maintenance:\n";
    if (params.getLubricationType() == "Grease") {
        schedule << " - Replace grease every 1000 hours\n";
    } else if (params.getLubricationType() == "Oil-Mist") {
        schedule << " - Check oil-mist system every 500 hours\n";
    } else {
        schedule << " - Monitor oil-air system every 300 hours\n";
    }
    schedule << " - Ensure no contamination in lubricant\n\n";

    schedule << "Vibration Monitoring:\n";
    schedule << " - Install vibration sensors for continuous monitoring\n";
    schedule << " - Check for anomalies every 100 hours\n";
    schedule << " - Maintain ISO 1940 G1 balance grade\n\n";

    schedule << "Alignment Check:\n";
    schedule << " - Verify alignment with laser tools every 500 hours\n";
    schedule << " - Ensure concentricity and parallelism\n\n";

    schedule << "General Maintenance:\n";
    schedule << " - Inspect spindle housing for cracks every 2000 hours\n";
    schedule << " - Dress grinding wheel every 50 hours to maintain geometry\n";
    schedule << " - Log performance trends for predictive maintenance\n";

    return schedule.str();
}

std::string SpindleSimulation::evaluateSpindleType(const SpindleParameters& params) const {
    if (params.getSpindleType() == "Motorized" && params.getMaxSpeed() > 15000)
        return "Motorized spindle optimal for high-speed precision grinding";
    if (params.getSpindleType() == "Belt-Driven" && params.getMaxSpeed() <= 8000)
        return "Belt-driven spindle cost-effective for high-torque applications";
    if (params.getSpindleType() == "Direct-Drive")
        return "Direct-drive spindle balances speed and torque effectively";
    return "Spindle type may not be optimal for specified parameters";
}

std::string SpindleSimulation::evaluateBearingPerformance(const SpindleParameters& params) const {
    if (params.getBearingType() == "Hybrid Ceramic" && params.getMaxSpeed() > 10000)
        return "Hybrid ceramic bearings optimal for high-speed, low-friction operation";
    if (params.getBearingType() == "Angular Contact")
        return "Angular contact bearings provide excellent rigidity for medium speeds";
    return "Bearing type may need review for optimal performance";
}

double SpindleSimulation::calculateRequiredPower(double wheelDiameter, int speed) const {
    double materialFactor = 1.2;
    return (wheelDiameter / 1000.0) * (speed / 1000.0) * 2.5 * materialFactor;
}

double SpindleSimulation::estimateTemperatureRise(const SpindleParameters& params) const {
    double baseTemp = params.getCoolingType() == "Liquid" ? 18.0 : 22.0;
    double speedFactor = params.getMaxSpeed() / 10000.0;
    double preloadFactor = params.getBearingPreload() / 500.0;
    return baseTemp + (speedFactor * 5.0) + (preloadFactor * 2.0);
}

double SpindleSimulation::estimateTemperatureRise(const SpindleParameters& params, double load) const {
    double baseTemp = params.getCoolingType() == "Liquid" ? 18.0 : 22.0;
    double speedFactor = params.getMaxSpeed() / 10000.0;
    double preloadFactor = params.getBearingPreload() / 500.0;
    double loadFactor = load / 1000.0;
    return baseTemp + (speedFactor * 5.0) + (preloadFactor * 2.0) + (loadFactor * 2.0);
}

double SpindleSimulation::calculateThermalExpansion(double tempRise) const {
    double shaftLength = 0.2;
    double thermalCoefficient = 12e-6;
    return shaftLength * thermalCoefficient * tempRise;
}

double SpindleSimulation::estimateVibration(const SpindleParameters& params) const {
    double baseVibration = params.getBearingType() == "Hybrid Ceramic" ? 0.4 : 0.6;
    double speedFactor = params.getMaxSpeed() / 10000.0;
    double alignmentFactor = params.getAlignmentTolerance() > 0.002 ? 1.2 : 1.0;
    double toolFactor = params.getToolInterface() == "HSK" ? 0.9 : 1.0;
    return baseVibration * speedFactor * alignmentFactor * toolFactor;
}

double SpindleSimulation::estimateVibration(const SpindleParameters& params, double load) const {
    double baseVibration = params.getBearingType() == "Hybrid Ceramic" ? 0.4 : 0.6;
    double speedFactor = params.getMaxSpeed() / 10000.0;
    double alignmentFactor = params.getAlignmentTolerance() > 0.002 ? 1.2 : 1.0;
    double toolFactor = params.getToolInterface() == "HSK" ? 0.9 : 1.0;
    double loadFactor = 1.0 + (load / 1000.0) * 0.5;
    return baseVibration * speedFactor * alignmentFactor * toolFactor * loadFactor;
}

double SpindleSimulation::calculateResonanceFrequency(const SpindleParameters& params) const {
    double stiffness = params.getBearingType() == "Hybrid Ceramic" ? 1.5e8 : 1.2e8;
    double mass = params.getWheelDiameter() / 1000.0 * 2.0;
    return std::sqrt(stiffness / mass) / (2 * M_PI);
}

double SpindleSimulation::estimateLoad(const SpindleParameters& params) const {
    return (params.getWheelDiameter() / 1000.0) * (params.getMaxSpeed() / 1000.0) * 100.0;
}

std::vector<double> SpindleSimulation::generateDynamicLoadProfile(const SpindleParameters& params, double duration, double loadFactor) const {
    std::vector<double> loadProfile;
    double baseLoad = estimateLoad(params) * loadFactor;
    double timeStep = 0.1;
    int steps = static_cast<int>(duration / timeStep);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < steps; ++i) {
        double time = i * timeStep;
        double variation = std::sin(2 * M_PI * time / 2.0) * 0.3;
        double load = baseLoad * (1.0 + variation);
        if (dist(rng) < 0.1) load *= 1.5;
        loadProfile.push_back(std::max(0.0, load));
    }
    return loadProfile;
}

double SpindleSimulation::calculateBearingL10Life(const SpindleParameters& params, const std::vector<double>& loadProfile) const {
    double C = params.getBearingType() == "Hybrid Ceramic" ? 50.0 : 40.0;
    double avgLoad = std::accumulate(loadProfile.begin(), loadProfile.end(), 0.0) / loadProfile.size();
    double P = (avgLoad + params.getBearingPreload()) / 1000.0;
    double lifeAdjustmentFactor = 1.0;
    if (params.getLubricationType() == "Grease") lifeAdjustmentFactor *= 0.8;
    else if (params.getLubricationType() == "Oil-Air") lifeAdjustmentFactor *= 1.2;
    if (params.getCoolingType() == "Liquid") lifeAdjustmentFactor *= 1.1;
    double L10 = std::pow(C / P, 3) * 1'000'000;
    double L10h = L10 / (60.0 * params.getMaxSpeed()) * lifeAdjustmentFactor;
    return std::max(1000.0, L10h);
}

double SpindleSimulation::calculateSpindleFatigueLife(const SpindleParameters& params, const std::vector<double>& loadProfile) const {
    double a = 20.0, b = 6.0, ultimateStrength = 800e6, shaftDiameter = 0.05;
    double sectionModulus = M_PI * std::pow(shaftDiameter, 3) / 32;
    double totalDamage = 0.0;

    for (double load : loadProfile) {
        double moment = load * 0.1;
        double stress = moment / sectionModulus;
        double logN = a - b * std::log10(stress / 1e6);
        double N = std::pow(10, logN);
        totalDamage += 1.0 / N;
    }

    double remainingLife = 1.0 - totalDamage;
    return std::max(0.0, std::min(1.0, remainingLife));
}

double SpindleSimulation::calculateWheelWear(const SpindleParameters& params, const std::vector<double>& loadProfile, double duration) const {
    double wearCoefficient = 1e-6;
    double wheelDiameter = params.getWheelDiameter() / 1000.0;
    double wheelThickness = 0.02;
    double avgLoad = std::accumulate(loadProfile.begin(), loadProfile.end(), 0.0) / loadProfile.size();
    double peripheralSpeed = M_PI * wheelDiameter * params.getMaxSpeed() / 60.0;
    double slidingDistance = peripheralSpeed * duration;
    double wearVolume = wearCoefficient * avgLoad * slidingDistance;
    double diameterReduction = wearVolume / (M_PI * wheelDiameter * wheelThickness * 1000.0);
    return std::min(diameterReduction, params.getWheelDiameter() * 0.2);
}

double SpindleSimulation::calculateWearInducedVibration(const SpindleParameters& params, double wear) const {
    double wheelDiameter = params.getWheelDiameter() / 1000.0;
    double wheelThickness = 0.02;
    double density = 2500.0;
    double wearVolume = wear * M_PI * wheelDiameter * wheelThickness * 1000.0;
    double imbalanceMass = density * wearVolume * 1e-9;
    double wheelMass = density * M_PI * std::pow(wheelDiameter / 2, 2) * wheelThickness;
    double eccentricity = (imbalanceMass * (wheelDiameter / 2)) / wheelMass;
    double omega = 2 * M_PI * params.getMaxSpeed() / 60.0;
    double imbalanceForce = imbalanceMass * std::pow(omega, 2) * eccentricity;
    double systemStiffness = 1e8;
    double vibrationAmplitude = imbalanceForce / systemStiffness * 1000.0;
    return std::min(vibrationAmplitude, 2.0);
}

void SpindleSimulation::generateHistoricalData() {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    for (int i = 0; i < 100; ++i) {
        double vibration = 0.2 + dist(rng) * 2.0;
        double temperature = 20.0 + dist(rng) * 30.0;
        double load = 500.0 + dist(rng) * 1500.0;
        double bearingLife = 1000.0 + dist(rng) * 49000.0;
        double spindleLife = dist(rng);
        double wheelWear = dist(rng) * 40.0;
        int label = (vibration > 1.0 || bearingLife < 5000 || spindleLife < 0.5 || wheelWear > 40.0 * 0.5) ? 1 : 0;
        historicalData.emplace_back(vibration, temperature, load, bearingLife, spindleLife, wheelWear, label);
    }
}

double SpindleSimulation::calculateEuclideanDistance(const DataPoint& p1, const DataPoint& p2) const {
    double vibDiff = (p1.vibration - p2.vibration) / 2.0;
    double tempDiff = (p1.temperature - p2.temperature) / 30.0; // Fixed typo: p2opentemperature -> p2.temperature
    double loadDiff = (p1.load - p2.load) / 1500.0;
    double bearingLifeDiff = (p1.bearingLife - p2.bearingLife) / 50000.0;
    double spindleLifeDiff = p1.spindleLife - p2.spindleLife;
    double wheelWearDiff = (p1.wheelWear - p2.wheelWear) / 40.0;

    return std::sqrt(
        vibDiff * vibDiff +
        tempDiff * tempDiff +
        loadDiff * loadDiff +
        bearingLifeDiff * bearingLifeDiff +
        spindleLifeDiff * spindleLifeDiff +
        wheelWearDiff * wheelWearDiff
    );
}

int SpindleSimulation::predictMaintenance(double vibration, double temperature, double load, double bearingLife, double spindleLife, double wheelWear) {
    if (historicalData.empty()) generateHistoricalData();

    DataPoint query(vibration, temperature, load, bearingLife, spindleLife, wheelWear, 0);
    std::vector<std::pair<double, int>> distances;
    for (const auto& data : historicalData) {
        double distance = calculateEuclideanDistance(query, data);
        distances.emplace_back(distance, data.label);
    }

    std::sort(distances.begin(), distances.end());
    int k = 3, yesCount = 0;
    for (int i = 0; i < k && i < distances.size(); ++i) {
        if (distances[i].second == 1) ++yesCount;
    }

    return yesCount > k / 2 ? 1 : 0;
}

// MOGA-related methods
SpindleParameters SpindleSimulation::generateRandomParameters() {
    SpindleParameters params;
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    // Continuous parameters
    params.setPowerRating(0.5 + dist(rng) * (50.0 - 0.5));
    params.setMaxSpeed(static_cast<int>(1000 + dist(rng) * (30000 - 1000)));
    params.setWheelDiameter(50.0 + dist(rng) * (1000.0 - 50.0));
    params.setBearingPreload(100.0 + dist(rng) * (2000.0 - 100.0));
    params.setAlignmentTolerance(0.0001 + dist(rng) * (0.01 - 0.0001));

    // Categorical parameters
    std::vector<std::string> spindleTypes = {"Belt-Driven", "Direct-Drive", "Motorized"};
    params.setSpindleType(spindleTypes[static_cast<int>(dist(rng) * spindleTypes.size())]);

    std::vector<std::string> bearingTypes = {"Angular Contact", "Hybrid Ceramic"};
    params.setBearingType(bearingTypes[static_cast<int>(dist(rng) * bearingTypes.size())]);

    std::vector<std::string> coolingTypes = {"Liquid", "Air"};
    params.setCoolingType(coolingTypes[static_cast<int>(dist(rng) * coolingTypes.size())]);

    std::vector<std::string> lubricationTypes = {"Grease", "Oil-Mist", "Oil-Air"};
    params.setLubricationType(lubricationTypes[static_cast<int>(dist(rng) * lubricationTypes.size())]);

    std::vector<std::string> toolInterfaces = {"Precision Collet", "Hydraulic Chuck", "HSK"};
    params.setToolInterface(toolInterfaces[static_cast<int>(dist(rng) * toolInterfaces.size())]);

    return params;
}

void SpindleSimulation::evaluateObjectives(Individual& ind, double duration, double loadFactor) {
    try {
        std::vector<double> loadProfile = generateDynamicLoadProfile(ind.params, duration, loadFactor);
        if (loadProfile.empty()) {
            throw std::runtime_error("Empty load profile generated");
        }
        double vibration = estimateVibration(ind.params);
        double tempRise = estimateTemperatureRise(ind.params);
        double bearingLife = calculateBearingL10Life(ind.params, loadProfile);
        double wheelWear = calculateWheelWear(ind.params, loadProfile, duration);
        double wearVibration = calculateWearInducedVibration(ind.params, wheelWear);
        double totalVibration = vibration + wearVibration;

        ind.objectives = {totalVibration, -bearingLife, tempRise}; // Minimize vibration, maximize bearing life (negated), minimize temperature
    } catch (const std::exception& e) {
        std::cerr << "Error in evaluateObjectives: " << e.what() << std::endl;
        ind.objectives = {1e10, -1e-10, 1e10}; // Assign worst-case objectives to prevent propagation
    }
}

bool SpindleSimulation::dominates(const Individual& a, const Individual& b) const {
    bool betterInAtLeastOne = false;
    for (size_t i = 0; i < a.objectives.size(); ++i) {
        if (a.objectives[i] > b.objectives[i]) return false; // a is worse in at least one objective
        if (a.objectives[i] < b.objectives[i]) betterInAtLeastOne = true; // a is better in at least one
    }
    return betterInAtLeastOne;
}

void SpindleSimulation::nonDominatedSorting(std::vector<Individual>& population) {
    std::ofstream log("optimization_log.txt", std::ios::app);
    log << "Starting nonDominatedSorting with population size: " << population.size() << std::endl;
    std::cout << "Starting nonDominatedSorting with population size: " << population.size() << std::endl;

    std::vector<std::vector<size_t>> fronts;
    std::vector<int> dominationCount(population.size(), 0);
    std::vector<std::vector<size_t>> dominatedBy(population.size());

    // Compute domination counts and dominated sets
    for (size_t i = 0; i < population.size(); ++i) {
        for (size_t j = 0; j < population.size(); ++j) {
            if (i == j) continue;
            if (dominates(population[i], population[j])) {
                dominatedBy[i].push_back(j);
            } else if (dominates(population[j], population[i])) {
                dominationCount[i]++;
            }
        }
        if (dominationCount[i] == 0) {
            population[i].rank = 1;
            if (fronts.empty()) fronts.push_back({});
            fronts[0].push_back(i);
        }
    }

    // Build subsequent fronts
    size_t frontIdx = 0;
    while (frontIdx < fronts.size() && !fronts[frontIdx].empty()) {
        std::vector<size_t> nextFront;
        for (size_t i : fronts[frontIdx]) {
            for (size_t j : dominatedBy[i]) {
                dominationCount[j]--;
                if (dominationCount[j] == 0) {
                    population[j].rank = frontIdx + 2;
                    nextFront.push_back(j);
                }
            }
        }
        if (!nextFront.empty()) fronts.push_back(nextFront);
        frontIdx++;
    }

    // Compute crowding distance for each front
    for (size_t f = 0; f < fronts.size(); ++f) {
        const auto& front = fronts[f];
        log << "Processing front " << f + 1 << " with " << front.size() << " individuals" << std::endl;
        if (front.size() <= 2) {
            for (size_t i : front) population[i].crowdingDistance = std::numeric_limits<double>::infinity();
            continue;
        }
        std::vector<size_t> sortedFront = front;
        for (size_t objIdx = 0; objIdx < population[front[0]].objectives.size(); ++objIdx) {
            std::sort(sortedFront.begin(), sortedFront.end(), [&](size_t a, size_t b) {
                return population[a].objectives[objIdx] < population[b].objectives[objIdx];
            });
            population[sortedFront.front()].crowdingDistance = std::numeric_limits<double>::infinity();
            population[sortedFront.back()].crowdingDistance = std::numeric_limits<double>::infinity();
            double objRange = population[sortedFront.back()].objectives[objIdx] - population[sortedFront.front()].objectives[objIdx];
            if (std::abs(objRange) < 1e-10) continue; // Skip if range is effectively zero
            for (size_t i = 1; i < sortedFront.size() - 1; ++i) {
                if (i + 1 < sortedFront.size() && i >= 1) {
                    population[sortedFront[i]].crowdingDistance +=
                        (population[sortedFront[i + 1]].objectives[objIdx] - population[sortedFront[i - 1]].objectives[objIdx]) / objRange;
                }
            }
        }
    }
    log << "Completed nonDominatedSorting, fronts created: " << fronts.size() << std::endl;
    std::cout << "Completed nonDominatedSorting, fronts created: " << fronts.size() << std::endl;
    log.close();
}

SpindleSimulation::Individual SpindleSimulation::crossover(const Individual& parent1, const Individual& parent2) {
    Individual offspring;
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double alpha = 0.5; // BLX-α parameter

    // Continuous parameters
    auto blend = [&](double p1, double p2, double min, double max) {
        double d = std::abs(p1 - p2);
        double lower = std::min(p1, p2) - alpha * d;
        double upper = std::max(p1, p2) + alpha * d;
        return std::max(min, std::min(max, lower + dist(rng) * (upper - lower)));
    };

    offspring.params.setPowerRating(blend(parent1.params.getPowerRating(), parent2.params.getPowerRating(), 0.5, 50.0));
    offspring.params.setMaxSpeed(static_cast<int>(blend(parent1.params.getMaxSpeed(), parent2.params.getMaxSpeed(), 1000, 30000)));
    offspring.params.setWheelDiameter(blend(parent1.params.getWheelDiameter(), parent2.params.getWheelDiameter(), 50.0, 1000.0));
    offspring.params.setBearingPreload(blend(parent1.params.getBearingPreload(), parent2.params.getBearingPreload(), 100.0, 2000.0));
    offspring.params.setAlignmentTolerance(blend(parent1.params.getAlignmentTolerance(), parent2.params.getAlignmentTolerance(), 0.0001, 0.01));

    // Categorical parameters (uniform crossover)
    offspring.params.setSpindleType(dist(rng) < 0.5 ? parent1.params.getSpindleType() : parent2.params.getSpindleType());
    offspring.params.setBearingType(dist(rng) < 0.5 ? parent1.params.getBearingType() : parent2.params.getBearingType());
    offspring.params.setCoolingType(dist(rng) < 0.5 ? parent1.params.getCoolingType() : parent2.params.getCoolingType());
    offspring.params.setLubricationType(dist(rng) < 0.5 ? parent1.params.getLubricationType() : parent2.params.getLubricationType());
    offspring.params.setToolInterface(dist(rng) < 0.5 ? parent1.params.getToolInterface() : parent2.params.getToolInterface());

    return offspring;
}

void SpindleSimulation::mutate(Individual& ind) {
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    double mutationProb = 0.1;

    // Continuous parameters (polynomial mutation)
    auto mutateContinuous = [&](double value, double min, double max, double eta = 20.0) {
        if (dist(rng) >= mutationProb) return value;
        double delta1 = (value - min) / (max - min);
        double delta2 = (max - value) / (max - min);
        double rand = dist(rng);
        double deltaq = (rand <= 0.5) ?
            std::pow(2.0 * rand, 1.0 / (eta + 1.0)) - 1.0 :
            1.0 - std::pow(2.0 * (1.0 - rand), 1.0 / (eta + 1.0));
        double delta = (deltaq < 0 ? delta1 : delta2) * deltaq;
        return std::max(min, std::min(max, value + delta * (max - min)));
    };

    ind.params.setPowerRating(mutateContinuous(ind.params.getPowerRating(), 0.5, 50.0));
    ind.params.setMaxSpeed(static_cast<int>(mutateContinuous(ind.params.getMaxSpeed(), 1000, 30000)));
    ind.params.setWheelDiameter(mutateContinuous(ind.params.getWheelDiameter(), 50.0, 1000.0));
    ind.params.setBearingPreload(mutateContinuous(ind.params.getBearingPreload(), 100.0, 2000.0));
    ind.params.setAlignmentTolerance(mutateContinuous(ind.params.getAlignmentTolerance(), 0.0001, 0.01));

    // Categorical parameters
    if (dist(rng) < mutationProb) {
        std::vector<std::string> spindleTypes = {"Belt-Driven", "Direct-Drive", "Motorized"};
        ind.params.setSpindleType(spindleTypes[static_cast<int>(dist(rng) * spindleTypes.size())]);
    }
    if (dist(rng) < mutationProb) {
        std::vector<std::string> bearingTypes = {"Angular Contact", "Hybrid Ceramic"};
        ind.params.setBearingType(bearingTypes[static_cast<int>(dist(rng) * bearingTypes.size())]);
    }
    if (dist(rng) < mutationProb) {
        std::vector<std::string> coolingTypes = {"Liquid", "Air"};
        ind.params.setCoolingType(coolingTypes[static_cast<int>(dist(rng) * coolingTypes.size())]);
    }
    if (dist(rng) < mutationProb) {
        std::vector<std::string> lubricationTypes = {"Grease", "Oil-Mist", "Oil-Air"};
        ind.params.setLubricationType(lubricationTypes[static_cast<int>(dist(rng) * lubricationTypes.size())]);
    }
    if (dist(rng) < mutationProb) {
        std::vector<std::string> toolInterfaces = {"Precision Collet", "Hydraulic Chuck", "HSK"};
        ind.params.setToolInterface(toolInterfaces[static_cast<int>(dist(rng) * toolInterfaces.size())]);
    }
}

std::string SpindleSimulation::optimizeSpindleArrangement(double duration, double loadFactor, int populationSize, int generations) {
    std::ofstream log("optimization_log.txt", std::ios::app);
    log << "Starting optimizeSpindleArrangement with duration: " << duration
        << ", loadFactor: " << loadFactor << ", populationSize: " << populationSize
        << ", generations: " << generations << std::endl;
    std::cout << "Starting optimizeSpindleArrangement with duration: " << duration
              << ", loadFactor: " << loadFactor << ", populationSize: " << populationSize
              << ", generations: " << generations << std::endl;

    try {
        // Validate inputs
        if (duration <= 0.0) {
            throw std::invalid_argument("Duration must be positive");
        }
        if (loadFactor < 0.5 || loadFactor > 2.0) {
            throw std::invalid_argument("Load factor must be between 0.5 and 2.0");
        }
        if (populationSize < 10 || generations < 1) {
            throw std::invalid_argument("Population size must be at least 10 and generations at least 1");
        }

        std::vector<Individual> population(populationSize);
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        // Initialize population
        log << "Initializing population..." << std::endl;
        std::cout << "Initializing population..." << std::endl;
        for (size_t i = 0; i < population.size(); ++i) {
            log << "Generating parameters for individual " << i + 1 << "/" << populationSize << std::endl;
            population[i].params = generateRandomParameters();
            log << "Evaluating objectives for individual " << i + 1 << "/" << populationSize << std::endl;
            evaluateObjectives(population[i], duration, loadFactor);
        }
        log << "Performing initial non-dominated sorting..." << std::endl;
        nonDominatedSorting(population);

        // Main loop
        for (int gen = 0; gen < generations; ++gen) {
            log << "Generation " << gen + 1 << "/" << generations << std::endl;
            std::cout << "Generation " << gen + 1 << "/" << generations << std::endl;
            std::vector<Individual> offspring;

            // Tournament selection and offspring creation
            log << "Performing tournament selection and creating offspring..." << std::endl;
            std::cout << "Performing tournament selection and creating offspring..." << std::endl;
            size_t offspringCount = 0;
            while (offspring.size() < populationSize) {
                size_t idx1 = static_cast<size_t>(dist(rng) * populationSize);
                size_t idx2 = static_cast<size_t>(dist(rng) * populationSize);
                if (idx1 >= population.size() || idx2 >= population.size()) {
                    log << "Error: Invalid tournament indices idx1=" << idx1 << ", idx2=" << idx2 << std::endl;
                    throw std::runtime_error("Invalid tournament indices");
                }
                Individual parent1 = population[idx1];
                Individual parent2 = population[idx2];
                log << "Selected parents idx1=" << idx1 << " (rank=" << parent1.rank << "), idx2=" << idx2 << " (rank=" << parent2.rank << ")" << std::endl;
                Individual offspringInd;
                if (parent1.rank < parent2.rank || (parent1.rank == parent2.rank && parent1.crowdingDistance > parent2.crowdingDistance)) {
                    offspringInd = crossover(parent1, parent2);
                } else {
                    offspringInd = crossover(parent2, parent1);
                }
                log << "Performing mutation on offspring " << offspringCount + 1 << std::endl;
                mutate(offspringInd);
                log << "Evaluating objectives for offspring " << offspringCount + 1 << std::endl;
                evaluateObjectives(offspringInd, duration, loadFactor);
                offspring.push_back(offspringInd);
                offspringCount++;
            }

            // Combine parent and offspring populations
            log << "Combining populations..." << std::endl;
            std::cout << "Combining populations..." << std::endl;
            std::vector<Individual> combined = population;
            combined.insert(combined.end(), offspring.begin(), offspring.end());
            if (combined.empty()) {
                log << "Error: Combined population is empty" << std::endl;
                throw std::runtime_error("Combined population is empty");
            }
            log << "Combined population size: " << combined.size() << std::endl;
            std::cout << "Combined population size: " << combined.size() << std::endl;

            // Recompute fronts for combined population
            log << "Recomputing fronts..." << std::endl;
            std::cout << "Recomputing fronts..." << std::endl;
            std::vector<std::vector<size_t>> fronts;
            std::vector<int> dominationCount(combined.size(), 0);
            std::vector<std::vector<size_t>> dominatedBy(combined.size());

            for (size_t i = 0; i < combined.size(); ++i) {
                for (size_t j = 0; j < combined.size(); ++j) {
                    if (i == j) continue;
                    if (dominates(combined[i], combined[j])) {
                        dominatedBy[i].push_back(j);
                    } else if (dominates(combined[j], combined[i])) {
                        dominationCount[i]++;
                    }
                }
                if (dominationCount[i] == 0) {
                    combined[i].rank = 1;
                    if (fronts.empty()) fronts.push_back({});
                    fronts[0].push_back(i);
                }
            }

            size_t frontIdx = 0;
            while (frontIdx < fronts.size() && !fronts[frontIdx].empty()) {
                std::vector<size_t> nextFront;
                for (size_t i : fronts[frontIdx]) {
                    for (size_t j : dominatedBy[i]) {
                        dominationCount[j]--;
                        if (dominationCount[j] == 0) {
                            combined[j].rank = frontIdx + 2;
                            nextFront.push_back(j);
                        }
                    }
                }
                if (!nextFront.empty()) fronts.push_back(nextFront);
                frontIdx++;
            }
            log << "Fronts recomputed, total fronts: " << fronts.size() << std::endl;
            std::cout << "Fronts recomputed, total fronts: " << fronts.size() << std::endl;

            // Compute crowding distance
            log << "Computing crowding distances..." << std::endl;
            std::cout << "Computing crowding distances..." << std::endl;
            for (size_t f = 0; f < fronts.size(); ++f) {
                const auto& front = fronts[f];
                log << "Processing front " << f + 1 << " with " << front.size() << " individuals" << std::endl;
                std::cout << "Processing front " << f + 1 << " with " << front.size() << " individuals" << std::endl;
                if (front.size() <= 2) {
                    for (size_t i : front) {
                        if (i < combined.size()) {
                            combined[i].crowdingDistance = std::numeric_limits<double>::infinity();
                        }
                    }
                    continue;
                }
                std::vector<size_t> sortedFront = front;
                for (size_t objIdx = 0; objIdx < combined[front[0]].objectives.size(); ++objIdx) {
                    std::sort(sortedFront.begin(), sortedFront.end(), [&](size_t a, size_t b) {
                        if (a >= combined.size() || b >= combined.size()) return false;
                        return combined[a].objectives[objIdx] < combined[b].objectives[objIdx];
                    });
                    if (!sortedFront.empty() && sortedFront.front() < combined.size()) {
                        combined[sortedFront.front()].crowdingDistance = std::numeric_limits<double>::infinity();
                    }
                    if (!sortedFront.empty() && sortedFront.back() < combined.size()) {
                        combined[sortedFront.back()].crowdingDistance = std::numeric_limits<double>::infinity();
                    }
                    if (sortedFront.size() <= 2) continue;
                    double objRange = combined[sortedFront.back()].objectives[objIdx] - combined[sortedFront.front()].objectives[objIdx];
                    if (std::abs(objRange) < 1e-10) continue;
                    for (size_t i = 1; i < sortedFront.size() - 1; ++i) {
                        if (i + 1 < sortedFront.size() && i >= 1 && sortedFront[i] < combined.size() &&
                            sortedFront[i + 1] < combined.size() && sortedFront[i - 1] < combined.size()) {
                            combined[sortedFront[i]].crowdingDistance +=
                                (combined[sortedFront[i + 1]].objectives[objIdx] - combined[sortedFront[i - 1]].objectives[objIdx]) / objRange;
                        } else {
                            log << "Warning: Skipping crowding distance for index " << i << " in front " << f + 1 << std::endl;
                        }
                    }
                }
            }

            // Populate next generation
            log << "Selecting next generation..." << std::endl;
            std::cout << "Selecting next generation..." << std::endl;
            population.clear();
            frontIdx = 0;
            while (frontIdx < fronts.size() && population.size() + fronts[frontIdx].size() <= static_cast<size_t>(populationSize)) {
                log << "Adding front " << frontIdx + 1 << " with " << fronts[frontIdx].size() << " individuals" << std::endl;
                std::cout << "Adding front " << frontIdx + 1 << " with " << fronts[frontIdx].size() << " individuals" << std::endl;
                for (size_t i : fronts[frontIdx]) {
                    if (i < combined.size()) {
                        population.push_back(combined[i]);
                    }
                }
                frontIdx++;
            }
            if (population.size() < static_cast<size_t>(populationSize) && frontIdx < fronts.size()) {
                log << "Partially adding front " << frontIdx + 1 << " to fill population" << std::endl;
                std::cout << "Partially adding front " << frontIdx + 1 << " to fill population" << std::endl;
                std::vector<size_t> sortedFront = fronts[frontIdx];
                std::sort(sortedFront.begin(), sortedFront.end(), [&](size_t a, size_t b) {
                    if (a >= combined.size() || b >= combined.size()) return false;
                    return combined[a].crowdingDistance > combined[b].crowdingDistance;
                });
                size_t remaining = populationSize - population.size();
                log << "Need to add " << remaining << " individuals from front " << frontIdx + 1 << " (size: " << sortedFront.size() << ")" << std::endl;
                for (size_t i = 0; i < remaining && i < sortedFront.size(); ++i) {
                    if (sortedFront[i] < combined.size()) {
                        population.push_back(combined[sortedFront[i]]);
                    }
                }
            }
            if (population.empty()) {
                log << "Error: Population is empty after selection" << std::endl;
                throw std::runtime_error("Population is empty after selection");
            }
            log << "New population size: " << population.size() << std::endl;
            std::cout << "New population size: " << population.size() << std::endl;
        }

        // Output Pareto front (rank 1 solutions)
        log << "Generating Pareto front output..." << std::endl;
        std::cout << "Generating Pareto front output..." << std::endl;
        std::stringstream report;
        report << std::fixed << std::setprecision(2);
        report << "=== Pareto-Optimal Spindle Arrangements ===\n\n";
        report << "Objectives: Minimize Vibration (mm/s), Maximize Bearing Life (hours), Minimize Temperature Rise (°C)\n\n";
        report << std::left;
        report << std::setw(12) << "Vibration" << std::setw(12) << "Bearing Life" << std::setw(12) << "Temp Rise"
               << std::setw(10) << "Power" << std::setw(10) << "Speed" << std::setw(12) << "Wheel Diam"
               << std::setw(10) << "Preload" << std::setw(12) << "Align Tol" << std::setw(15) << "Spindle Type"
               << std::setw(15) << "Bearing Type" << std::setw(10) << "Cooling" << std::setw(12) << "Lubrication"
               << std::setw(15) << "Tool Interface" << "\n";

        int paretoCount = 0;
        for (const auto& ind : population) {
            if (ind.rank != 1) continue;
            report << std::setw(12) << ind.objectives[0] // Vibration
                   << std::setw(12) << -ind.objectives[1] // Bearing Life
                   << std::setw(12) << ind.objectives[2] // Temperature Rise
                   << std::setw(10) << ind.params.getPowerRating()
                   << std::setw(10) << ind.params.getMaxSpeed()
                   << std::setw(12) << ind.params.getWheelDiameter()
                   << std::setw(10) << ind.params.getBearingPreload()
                   << std::setw(12) << ind.params.getAlignmentTolerance()
                   << std::setw(15) << ind.params.getSpindleType()
                   << std::setw(15) << ind.params.getBearingType()
                   << std::setw(10) << ind.params.getCoolingType()
                   << std::setw(12) << ind.params.getLubricationType()
                   << std::setw(15) << ind.params.getToolInterface()
                   << "\n";
            paretoCount++;
        }
        report << "\nTotal Pareto-optimal solutions found: " << paretoCount << "\n";
        log << "Optimization complete, found " << paretoCount << " Pareto-optimal solutions" << std::endl;
        std::cout << "Optimization complete, found " << paretoCount << " Pareto-optimal solutions" << std::endl;
        log.close();
        return report.str();
    } catch (const std::exception& e) {
        log << "Error in optimizeSpindleArrangement: " << e.what() << std::endl;
        std::cerr << "Error in optimizeSpindleArrangement: " << e.what() << std::endl;
        log.close();
        return "Error: Optimization failed - " + std::string(e.what()) + "\n";
    } catch (...) {
        log << "Unknown error in optimizeSpindleArrangement" << std::endl;
        std::cerr << "Unknown error in optimizeSpindleArrangement" << std::endl;
        log.close();
        return "Error: Optimization failed - Unknown error\n";
    }
}