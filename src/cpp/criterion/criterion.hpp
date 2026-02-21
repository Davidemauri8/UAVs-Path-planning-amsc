#include <cmath>    
#include <cstdlib>  
#include <ctime>   
 
class TransitionCriterion {

public:
    virtual bool 
        accept(double delta_e, int step, double temperature) = 0;

};

class MetropolisCriterion: public TransitionCriterion {

public:
    //MetropolisCriterion() = default;
    bool accept(double delta_e, int step, double temperature) override {
        if (delta_e < 0.0) {
            return true;
        } else {
            double prob = std::exp(-delta_e / temperature);
            double r = (double)std::rand() / RAND_MAX;
            return r < prob;
        }  
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

public:
    GSACriterion(double q) : q_parameter(q) {}

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
            
            double r = (double)std::rand() / RAND_MAX;
            return r < prob;
        }
    }
};


