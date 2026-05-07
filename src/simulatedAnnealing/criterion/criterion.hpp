#pragma once
#ifndef CRITERION_HPP
#define CRITERION_HPP

#include <cmath>    
#include <cstdlib>  
#include <ctime>   
#include <random>

 
class TransitionCriterion {

public:
    virtual bool accept(double delta_e, int step, double temperature) = 0;
    virtual void seed(long seed) {
        // An empty implementations for criterions which do not require
        // stochasticity
    }
};

class MetropolisCriterion: public TransitionCriterion {


    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<double> distrib;

public:

    MetropolisCriterion(): gen(rd()), distrib(0, 1.0) { }

    bool accept(double delta_e, int step, double temperature) override {
        if (delta_e < 0.0) {
            return true;
        } else {
            double prob = std::exp(-delta_e / temperature);
            double r = distrib(gen);
            return r < prob;
        }  
    }

    void seed(long seed) override {
        gen.seed(seed);
    }
};


// Threshold Acceptance Criterion
class TACCriterion: public TransitionCriterion {


public:
    
    TACCriterion(double thresh) : threshold(thresh) {}
    
    bool accept(double delta_e, int step, double temperature) override { 
        (void) temperature; // Mark the variable as unused
        if (delta_e <= threshold) {
            return true;
        } else {
            return false;
        }
    }

    const double threshold;

};
// Generalized Simulated Annealing
class GSACriterion: public TransitionCriterion {

private:
    const double q_parameter; 

    std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<double> distrib;

public:
    GSACriterion(double q) : q_parameter(q), gen(rd()), distrib(0, 1.0) {}

    bool accept(double delta_e, int step, double temperature) override {
        if (delta_e < 0.0) {
            return true;
        } else {
            double base = 1.0 - (q_parameter - 1.0) * (delta_e / temperature);
            
            if (base <= 0.0) { 
                return false;
            }
            double exponent = 1.0 / (q_parameter - 1.0);
            double prob = std::pow(base, exponent);
            
            double r = distrib(gen);
            return r < prob;
        }
    }

    void seed(long seed) override {
        gen.seed(seed);
    }

};

#endif