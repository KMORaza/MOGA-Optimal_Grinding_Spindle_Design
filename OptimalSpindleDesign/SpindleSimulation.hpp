#ifndef SPINDLE_SIMULATION_HPP
#define SPINDLE_SIMULATION_HPP

#include "SpindleParameters.hpp"
#include <vector>
#include <string>
#include <random>

class SpindleSimulation {
private:
    struct DataPoint {
        double vibration;
        double temperature;
        double load;
        double bearingLife;
        double spindleLife;
        double wheelWear;
        int label; // 1 = maintenance needed, 0 = no maintenance
        DataPoint(double vib, double temp, double ld, double bLife, double sLife, double wWear, int lbl)
            : vibration(vib), temperature(temp), load(ld), bearingLife(bLife), spindleLife(sLife), wheelWear(wWear), label(lbl) {}
    };

    struct SimulationScenario {
        std::string name;
        double speedFactor;
        double loadFactor;
        double duration;
        SimulationScenario(const std::string& n, double sf, double lf, double d)
            : name(n), speedFactor(sf), loadFactor(lf), duration(d) {}
    };

    // MOGA-related structures
    struct Individual {
        SpindleParameters params;
        std::vector<double> objectives; // [vibration, -bearingLife, temperature]
        int rank;
        double crowdingDistance;
        Individual() : rank(0), crowdingDistance(0.0) {}
    };

    static std::vector<DataPoint> historicalData;
    mutable std::mt19937 rng; // Mutable to allow use in const methods

    std::string validateParameters(const SpindleParameters& params) const;
    std::string evaluateSpindleType(const SpindleParameters& params) const;
    std::string evaluateBearingPerformance(const SpindleParameters& params) const;
    double calculateThermalExpansion(double tempRise) const;
    double calculateResonanceFrequency(const SpindleParameters& params) const;
    std::string runSimulationStage(const SpindleParameters& params, const SimulationScenario& scenario);
    std::string generateComprehensiveReport(const SpindleParameters& params, const std::vector<SimulationScenario>& scenarios);
    void generateHistoricalData();
    double calculateEuclideanDistance(const DataPoint& p1, const DataPoint& p2) const;

    // MOGA-related methods
    void evaluateObjectives(Individual& ind, double duration, double loadFactor);
    bool dominates(const Individual& a, const Individual& b) const;
    void nonDominatedSorting(std::vector<Individual>& population);
    double calculateCrowdingDistance(const std::vector<Individual>& front, size_t objIdx) const;
    Individual crossover(const Individual& parent1, const Individual& parent2);
    void mutate(Individual& ind);
    SpindleParameters generateRandomParameters();

public:
    SpindleSimulation();
    std::string simulate(const SpindleParameters& params);
    std::string simulateTimeBased(const SpindleParameters& params, double duration);
    std::string generateMaintenanceSchedule(const SpindleParameters& params);
    double calculateRequiredPower(double wheelDiameter, int speed) const;
    double estimateTemperatureRise(const SpindleParameters& params) const;
    double estimateTemperatureRise(const SpindleParameters& params, double load) const;
    double estimateVibration(const SpindleParameters& params) const;
    double estimateVibration(const SpindleParameters& params, double load) const;
    double estimateLoad(const SpindleParameters& params) const;
    std::vector<double> generateDynamicLoadProfile(const SpindleParameters& params, double duration, double loadFactor) const;
    double calculateBearingL10Life(const SpindleParameters& params, const std::vector<double>& loadProfile) const;
    double calculateSpindleFatigueLife(const SpindleParameters& params, const std::vector<double>& loadProfile) const;
    double calculateWheelWear(const SpindleParameters& params, const std::vector<double>& loadProfile, double duration) const;
    double calculateWearInducedVibration(const SpindleParameters& params, double wear) const;
    int predictMaintenance(double vibration, double temperature, double load, double bearingLife, double spindleLife, double wheelWear);
    std::string optimizeSpindleArrangement(double duration, double loadFactor, int populationSize, int generations);
};

#endif // SPINDLE_SIMULATION_HPP