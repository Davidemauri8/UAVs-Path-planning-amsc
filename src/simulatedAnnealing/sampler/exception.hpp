#pragma once
#ifndef EXCEPTION_HPP
#define EXCEPTION_HPP

#include <stdexcept>
#include <string>

namespace sa {

    /*
     * Eccezione lanciata quando un sampler non riesce ad estrarre
     * un punto valido dopo un numero massimo di tentativi.
     */
    class SamplerFailure : public std::runtime_error {
    public:
        explicit SamplerFailure(const std::string& msg)
            : std::runtime_error("SamplerFailure: " + msg) {}
    };

    class NonLocalWrongUse : public std::runtime_error {
    public:
        explicit NonLocalWrongUse()
            : std::runtime_error("Non Local Sampler used in a wrong way! You are passing a probability that force a Local Sampler behaviour" ) {}
    };

    class LocalWrongUse : public std::runtime_error {
    public:
        explicit LocalWrongUse()
            : std::runtime_error("Local Sampler used in a wrong way! You are passing a probability that force a Non Local Sampler behaviour") {}
    };


}

#endif // EXCEPTION_HPP