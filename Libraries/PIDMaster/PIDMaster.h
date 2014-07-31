/**********************************************************************************************
* Arduino PIDMaster Library - Version 1.0
* v1.0 by Dan Barzilay <barzilaydn@gmail.com>
*
* This Library is licensed under a GPLv3 License
**********************************************************************************************/
#ifndef PIDMaster_h
#define PIDMaster_h
#define LIBRARY_VERSION	1.0.0

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <TimerOne.h>
#include <PID_v2.h>
#include <stdarg.h>

class PIDMaster
{
public:
    void PIDMaster(PID*, ... ); //Constructor
    void Attach(PID*, ... );
    void Start();
    void Stop();
    void Resume();
private:
    void Compute();
    unsigned long GCD(unsigned long u, unsigned long v);
    
    volatile Vector<PID*> children;        //Pointer to use array of PID instances.
    volatile unsigned long gcdSampleTime;  //in microseconds.
    volatile unsigned long maxSampleTime;  //in microseconds.
    unsigned int count;
};

// Minimal class to replace std::vector
template<typename Data>
class Vector {
    size_t d_size; // Stores no. of actually stored objects
    size_t d_capacity; // Stores allocated capacity
    Data *d_data; // Stores data
public:
    Vector() : d_size(0), d_capacity(0), d_data(0) {}; // Default constructor
    Vector(Vector const &other) : d_size(other.d_size), d_capacity(other.d_capacity), d_data(0) { d_data = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(d_data, other.d_data, d_size*sizeof(Data)); }; // Copy constuctor
    ~Vector() { free(d_data); }; // Destructor
    Vector &operator=(Vector const &other) { free(d_data); d_size = other.d_size; d_capacity = other.d_capacity; d_data = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(d_data, other.d_data, d_size*sizeof(Data)); return *this; }; // Needed for memory management
    void push_back(Data const &x) { if (d_capacity == d_size) resize(); d_data[d_size++] = x; }; // Adds new value. If needed, allocates more space
    size_t size() const { return d_size; }; // Size getter
    Data const &operator[](size_t idx) const { return d_data[idx]; }; // Const getter
    Data &operator[](size_t idx) { return d_data[idx]; }; // Changeable getter
private:
    void resize() { d_capacity = d_capacity ? d_capacity*2 : 1; Data *newdata = (Data *)malloc(d_capacity*sizeof(Data)); memcpy(newdata, d_data, d_size * sizeof(Data)); free(d_data); d_data = newdata; };// Allocates double the old space
};

#endif