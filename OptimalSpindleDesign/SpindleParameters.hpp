#ifndef SPINDLE_PARAMETERS_HPP
#define SPINDLE_PARAMETERS_HPP

#include <string>

struct SpindleParameters {
    std::string spindleType;
    double powerRating;
    int maxSpeed;
    double wheelDiameter;
    std::string bearingType;
    double bearingPreload;
    std::string coolingType;
    std::string lubricationType;
    std::string toolInterface;
    double alignmentTolerance;

    // Default constructor
    SpindleParameters() : powerRating(0.0), maxSpeed(0), wheelDiameter(0.0), bearingPreload(0.0), alignmentTolerance(0.0) {}

    // Setters
    void setSpindleType(const std::string& type) { 
        spindleType = type; 
    }
    void setPowerRating(double rating) { 
        powerRating = rating; 
    }
    void setMaxSpeed(int speed) { 
        maxSpeed = speed; 
    }
    void setWheelDiameter(double diameter) { 
        wheelDiameter = diameter; 
    }
    void setBearingType(const std::string& type) { 
        bearingType = type; 
    }
    void setBearingPreload(double preload) { 
        bearingPreload = preload; 
    }
    void setCoolingType(const std::string& type) { 
        coolingType = type; 
    }
    void setLubricationType(const std::string& type) { 
        lubricationType = type; 
    }
    void setToolInterface(const std::string& interface) { 
        toolInterface = interface; 
    }
    void setAlignmentTolerance(double tolerance) { 
        alignmentTolerance = tolerance; 
    }

    // Getters
    std::string getSpindleType() const { 
        return spindleType; 
    }
    double getPowerRating() const { 
        return powerRating; 
    }
    int getMaxSpeed() const { 
        return maxSpeed; 
    }
    double getWheelDiameter() const { 
        return wheelDiameter; 
    }
    std::string getBearingType() const { 
        return bearingType; 
    }
    double getBearingPreload() const { 
        return bearingPreload; 
    }
    std::string getCoolingType() const { 
        return coolingType; 
    }
    std::string getLubricationType() const { 
        return lubricationType; 
    }
    std::string getToolInterface() const { 
        return toolInterface; 
    }
    double getAlignmentTolerance() const { 
        return alignmentTolerance; 
    }
};

#endif // SPINDLE_PARAMETERS_HPP