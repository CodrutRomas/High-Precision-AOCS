#include "DataExporter.h"
#include <iostream>

DataExporter::DataExporter(const std::string& filename) : isFirstWrite(true) {
    file.open(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open file " << filename << std::endl;
    }
}

DataExporter::~DataExporter() {
    if (file.is_open()) {
        file.close();
    }
}

void DataExporter::writeHeader() {
    if (!file.is_open()) return;

    file << "time,q_w,q_x,q_y,q_z,omega_x,omega_y,omega_z,";
    file << "gravity_torque_x,gravity_torque_y,gravity_torque_z,";
    file << "magnetic_torque_x,magnetic_torque_y,magnetic_torque_z,";
    file << "atmospheric_torque_x,atmospheric_torque_y,atmospheric_torque_z,";
    file << "solar_torque_x,solar_torque_y,solar_torque_z,";
    file << "total_perturbation_x,total_perturbation_y,total_perturbation_z,";
    file << "control_torque_x,control_torque_y,control_torque_z,";
    file << "total_torque_x,total_torque_y,total_torque_z" << std::endl;
}

void DataExporter::writeData(double time,
    const Quaternion& q,
    const Vector3& omega,
    const Vector3& gravityTorque,
    const Vector3& magneticTorque,
    const Vector3& atmosphericTorque,
    const Vector3& solarTorque,
    const Vector3& totalPerturbation,
    const Vector3& controlTorque,
    const Vector3& totalTorque) {
    if (!file.is_open()) return;

    if (isFirstWrite) {
        writeHeader();
        isFirstWrite = false;
    }

    // Use getters instead of direct member access
    file << time << "," << q.getW() << "," << q.getX() << "," << q.getY() << "," << q.getZ() << ",";
    file << omega.getX() << "," << omega.getY() << "," << omega.getZ() << ",";
    file << gravityTorque.getX() << "," << gravityTorque.getY() << "," << gravityTorque.getZ() << ",";
    file << magneticTorque.getX() << "," << magneticTorque.getY() << "," << magneticTorque.getZ() << ",";
    file << atmosphericTorque.getX() << "," << atmosphericTorque.getY() << "," << atmosphericTorque.getZ() << ",";
    file << solarTorque.getX() << "," << solarTorque.getY() << "," << solarTorque.getZ() << ",";
    file << totalPerturbation.getX() << "," << totalPerturbation.getY() << "," << totalPerturbation.getZ() << ",";
    file << controlTorque.getX() << "," << controlTorque.getY() << "," << controlTorque.getZ() << ",";
    file << totalTorque.getX() << "," << totalTorque.getY() << "," << totalTorque.getZ() << std::endl;
}

void DataExporter::close() {
    if (file.is_open()) {
        file.close();
    }
}