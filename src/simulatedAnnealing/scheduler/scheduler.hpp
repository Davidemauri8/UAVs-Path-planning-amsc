#pragma once
#ifndef PARAM_SCHEDULER_HPP
#define PARAM_SCHEDULER_HPP

#include <stdexcept>

class ParamScheduler {
public:
   // Updates scheduler parameters at current step
   virtual void update(int curr_step) = 0;
  
   // Returns number of stabilization iterations at current temperature
   virtual long stab_it() const = 0;
  
   // Returns current temperature value
   virtual double temp() const = 0;
  
   virtual ~ParamScheduler() = default;
};


// Linear Cooling Schedule: T(k+1) = T(k) - delta
// Temperature decreases by a constant amount at each step
class LinearScheduler : public ParamScheduler {
private:
   double current_temp;
   double initial_temp;
   double temp_decrement;
   double min_temp;
   long stabilization_iterations;
  
public:
   LinearScheduler(double T0, double delta, double Tmin, long stab_it)
       : initial_temp(T0)
       , current_temp(T0)
       , temp_decrement(delta)
       , min_temp(Tmin)
       , stabilization_iterations(stab_it)
   {
       if (T0 < 0.0 || delta <= 0.0 || Tmin < 0.0 || stab_it  <= 0) {
           throw std::invalid_argument("Scheduler parameters must be positive");
       }
       if (Tmin > T0) {
           throw std::invalid_argument("Minimum temperature must be < initial temperature");
       }
   }
  
   // Allow for a purely linear scheduling over a fixed amount of steps, to be used
   // in combination with temp_zero stopping criterion
   LinearScheduler(double T0,
       long steps, long stab_it
   ) : LinearScheduler(T0, T0 / steps, 0.0, stab_it)
   { }

   void update(int curr_step) override {
       current_temp -= temp_decrement;
       if (current_temp < min_temp) {
           current_temp = min_temp;
       }
   }
  
   long stab_it() const override {
       return stabilization_iterations;
   }
  
   double temp() const override {
       return current_temp;
   }
  
   void reset() {
       current_temp = initial_temp;
   }
};


// Geometric/Exponential Cooling Schedule: T(k+1) = alpha * T(k)
// Temperature is multiplied by factor alpha < 1 at each step

class ExponentialScheduler : public ParamScheduler {
private:
   double current_temp;
   double initial_temp;
   double cooling_rate;
   double min_temp;
   long stabilization_iterations;
  
public:
   ExponentialScheduler(double T0,
        double Tmin,
        long stab_it,
        double alpha = 0.95
    )
       : initial_temp(T0)
       , current_temp(T0)
       , cooling_rate(alpha)
       , min_temp(Tmin)
       , stabilization_iterations(stab_it)
   {
       if (T0 <= 0.0 || Tmin <= 0.0 || stab_it <= 0) {
           throw std::invalid_argument("Scheduler parameters must be positive");
       }
       if (alpha <= 0.0 || alpha >= 1.0) {
           throw std::invalid_argument("Cooling rate alpha must be in (0, 1)");
       }
       if (Tmin > T0) {
           throw std::invalid_argument("Minimum temperature must be < initial temperature");
       }
   }
  
   void update(int curr_step) override {
       current_temp *= cooling_rate;
       if (current_temp < min_temp) {
           current_temp = min_temp;
       }
   }
  
   long stab_it() const override {
       return stabilization_iterations;
   }
  
   double temp() const override {
       return current_temp;
   }
  
   void reset() {
       current_temp = initial_temp;
   }
};

#endif
