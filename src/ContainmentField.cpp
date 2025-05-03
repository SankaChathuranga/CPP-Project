#include "../include/ContainmentField.h"
#include "../include/Particle.h"
#include "../include/Config.h"
#include <cmath>
#include <algorithm>
#include <numeric>

ContainmentField::ContainmentField(const Config& config)
    : size(config.field_size), fieldStrength(config.initial_strength), decayRate(config.initial_decay_rate), GRID_SIZE(config.field_grid_size), fieldEnergy(0.0) {
    initializeField();
}

ContainmentField::~ContainmentField() {
    
}

void ContainmentField::initializeField() {
    fieldData.resize(GRID_SIZE * GRID_SIZE, fieldStrength);
}

double ContainmentField::getContainmentForce(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Calculate distance from center
    double distanceFromCenter = std::sqrt(x*x + y*y);
    
    // Calculate distance from boundary
    double distanceFromBoundary = size/2.0 - distanceFromCenter;
    
    // Force increases as particle approaches boundary
    if (distanceFromBoundary <= 0) {
        return fieldStrength; // Maximum force at boundary
    }
    
    // Force decreases linearly with distance from boundary
    return fieldStrength * (1.0 - distanceFromBoundary / (size/2.0));
}

bool ContainmentField::isParticleContained(const Particle& particle) const {
    double x = particle.getX();
    double y = particle.getY();
    
    // Check if particle is within the square boundary
    return std::abs(x) <= size/2.0 && std::abs(y) <= size/2.0;
}

void ContainmentField::update(double dt) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    for (size_t i = 0; i < fieldData.size(); ++i) {
        fieldData[i] *= (1.0 - decayRate * dt);
    }
    fieldEnergy = std::accumulate(fieldData.begin(), fieldData.end(), 0.0);
}

void ContainmentField::setFieldStrength(double strength) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    fieldStrength = strength;
    // Update all grid points
    for (auto& value : fieldData) {
        value = strength;
    }
}

double ContainmentField::getFieldStrength() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldStrength;
}

void ContainmentField::setDecayRate(double rate) {
    std::lock_guard<std::mutex> lock(fieldMutex);
    decayRate = rate;
}

double ContainmentField::getDecayRate() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return decayRate;
}

double ContainmentField::getSize() const {
    return size;
}

double ContainmentField::getFieldEnergy() const {
    std::lock_guard<std::mutex> lock(fieldMutex);
    return fieldEnergy;
} 