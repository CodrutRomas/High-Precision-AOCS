#pragma once
#include <fstream>
#include <string>
#include <vector>
#include "Vector3.h"
#include "Quaternion.h"

class DataExporter {
private:
    std::ofstream file;
    bool isFirstWrite;

public:
    DataExporter(const std::string& filename);
    ~DataExporter();

    // Write header for CSV file
    void writeHeader();

    // Write data row to CSV
    void writeData(double time,
        const Quaternion& q,
        const Vector3& omega,
        const Vector3& gravityTorque,
        const Vector3& magneticTorque,
        const Vector3& atmosphericTorque,
        const Vector3& solarTorque,
        const Vector3& totalPerturbation,
        const Vector3& controlTorque,
        const Vector3& totalTorque);

    // Close file
    void close();
};