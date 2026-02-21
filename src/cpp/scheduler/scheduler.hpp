#pragma once
#ifndef _PARAM_SCHEDULER
#define _PARAM_SCHEDULER

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
   LinearScheduler(double T0 = 100.0,
                  double delta = 0.5,
                  double Tmin = 0.01,
                  long L = 1000)
       : initial_temp(T0)
       , current_temp(T0)
       , temp_decrement(delta)
       , min_temp(Tmin)
       , stabilization_iterations(L)
   {
       if (T0 <= 0.0 || delta <= 0.0 || Tmin <= 0.0 || L <= 0) {
           throw std::invalid_argument("Scheduler parameters must be positive");
       }
       if (Tmin >= T0) {
           throw std::invalid_argument("Minimum temperature must be < initial temperature");
       }
   }
  
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
// Example: T0=100, alpha=0.95 → 100, 95, 90.25, 85.74, ...
class ExponentialScheduler : public ParamScheduler {
private:
   double current_temp;
   double initial_temp;
   double cooling_rate;
   double min_temp;
   long stabilization_iterations;
  
public:
   ExponentialScheduler(double T0 = 100.0,
                       double alpha = 0.95,
                       double Tmin = 0.01,
                       long L = 1000)
       : initial_temp(T0)
       , current_temp(T0)
       , cooling_rate(alpha)
       , min_temp(Tmin)
       , stabilization_iterations(L)
   {
       if (T0 <= 0.0 || Tmin <= 0.0 || L <= 0) {
           throw std::invalid_argument("Scheduler parameters must be positive");
       }
       if (alpha <= 0.0 || alpha >= 1.0) {
           throw std::invalid_argument("Cooling rate alpha must be in (0, 1)");
       }
       if (Tmin >= T0) {
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
