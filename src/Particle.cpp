#include "../include/Particle.h"
#include <cmath>
#include <thread>
#include <chrono>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
}

Particle::~Particle() {
}

double Particle::getX() const {
    return x;
}

double Particle::getY() const {
    return y;
}

void Particle::setPosition(double newX, double newY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    x = newX;
    y = newY;
}

double Particle::getVX() const {
    return vx;
}

double Particle::getVY() const {
    return vy;
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    return energy;
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY;
}

void Particle::setEnergy(double newEnergy) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(newEnergy, MAX_ENERGY);
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY);
}

void Particle::collide(Particle& other) {
    std::lock_guard<std::mutex> lock1(particleMutex);
    std::lock_guard<std::mutex> lock2(other.particleMutex);
    
    // Calculate collision response using conservation of momentum and energy
    double total_mass = 2.0; // Assuming equal mass particles
    double vx_avg = (vx + other.vx) / 2.0;
    double vy_avg = (vy + other.vy) / 2.0;
    
    // Elastic collision
    vx = 2 * vx_avg - vx;
    vy = 2 * vy_avg - vy;
    other.vx = 2 * vx_avg - other.vx;
    other.vy = 2 * vy_avg - other.vy;
    
    // Energy transfer
    double energy_avg = (energy + other.energy) / 2.0;
    energy = energy_avg;
    other.energy = energy_avg;
}

bool Particle::isColliding(const Particle& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    return distance < (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
}
