/*
	Drivetrain Simulator for Shear Force Config
	Developed by Kaden Dadabhoy
*/

#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>
#include <cassert>
#include <filesystem>

using namespace std;
namespace fs = std::filesystem;


// Structure to hold InputFile data
struct Config
{
	double simTime = 5.0;
	double dt = 0.002;
	double v_batt = -1;
	double gearboxReduction = -1;
	double frontWheelInches = -1;
	double frontPulleyTeeth = -1;
	double motorPulleyFront = -1;
	double rearWheelInches = -1;
	double rearPulleyTeeth = -1;
	double motorPulleyRear = -1;
	double speedFactor = 0.85;
	double drivetrainEfficiency = 0.80;
	double kv = -1;
	double currentLimit = -1;
	double mu = -1;
	double totalWeightLb = -1;
};


/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void Simulate(const Config& cfg, string csv);
double getValueAfterColon(const string& line);
void DisplayConfig(const Config& cfg);

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
int main()
{
	bool keepRunning = true;

	while (keepRunning)
	{
		string fileName, csvName;
		Config cfg;

		system("cls"); // Clears the screen for a fresh run (Windows)
		cout << "=========================================================" << endl;
		cout << "   VERY ADVANCED DRIVETRAIN CALCULATOR - SHEAR FORCE    " << endl;
		cout << "                  By: Kaden Dadabhoy                    " << endl;
		cout << "=========================================================" << endl;
		cout << "Current Folder: " << fs::current_path().string() << endl;

		cout << "\nStep 1: Enter Input Filename (Default: DrivetrainInputeTemplate.txt): ";
		getline(cin, fileName);
		if (fileName.empty()) fileName = "DrivetrainInputeTemplate.txt";
		fileName.erase(remove(fileName.begin(), fileName.end(), '\"'), fileName.end());

		ifstream file(fileName);
		if (!file.is_open())
		{
			cerr << "\n[!] ERROR: Could not find '" << fileName << "'. Check the folder!" << endl;
		}
		else
		{
			// Parsing Logic
			string line;
			while (getline(file, line))
			{
				if (line.find("TotalSimulationTime") != string::npos) cfg.simTime = getValueAfterColon(line);
				else if (line.find("TimeStep") != string::npos) cfg.dt = getValueAfterColon(line);
				else if (line.find("Gearbox Reduction") != string::npos) cfg.gearboxReduction = getValueAfterColon(line);
				else if (line.find("Front Wheel Diameter") != string::npos) cfg.frontWheelInches = getValueAfterColon(line);
				else if (line.find("Front Pulley Teeth") != string::npos && line.find("Motor") == string::npos) cfg.frontPulleyTeeth = getValueAfterColon(line);
				else if (line.find("Motor Pulley Corresponding to Front") != string::npos) cfg.motorPulleyFront = getValueAfterColon(line);
				else if (line.find("Rear Wheel Diameter") != string::npos) cfg.rearWheelInches = getValueAfterColon(line);
				else if (line.find("Rear Pulley Teeth") != string::npos && line.find("Motor") == string::npos) cfg.rearPulleyTeeth = getValueAfterColon(line);
				else if (line.find("Motor Pulley Corresponding to Rear") != string::npos) cfg.motorPulleyRear = getValueAfterColon(line);
				else if (line.find("Speed_Factor") != string::npos) cfg.speedFactor = getValueAfterColon(line);
				else if (line.find("Drivetrain_Efficiency") != string::npos) cfg.drivetrainEfficiency = getValueAfterColon(line);
				else if (line.find("Kv") != string::npos) cfg.kv = getValueAfterColon(line);
				else if (line.find("Current Limit") != string::npos) cfg.currentLimit = getValueAfterColon(line);
				else if (line.find("Coefficient of Friction") != string::npos) cfg.mu = getValueAfterColon(line);
				else if (line.find("Total Weight") != string::npos) cfg.totalWeightLb = getValueAfterColon(line);
				else if (line.find("Battery Voltage (V):") != string::npos) cfg.v_batt = getValueAfterColon(line);
			}
			file.close();

			DisplayConfig(cfg);

			cout << "Step 2: Enter name for CSV results (Ex: run1.csv) or press ENTER to skip: ";
			getline(cin, csvName);

			Simulate(cfg, csvName);
		}

		cout << "\nSimulation Cycle Finished." << endl;
		cout << "Would you like to run again? (y/n): ";
		string choice;
		getline(cin, choice);
		if (choice != "y" && choice != "Y") keepRunning = false;
	}

	return 0;
}



/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////
void Simulate(const Config& cfg, string csvName)
{
	const double inches_to_meters = 0.0254;
	const double PI = 3.14159265;
	const double g = 9.80665;

	// Derived Physical Constants
	double mass_kg_side = (cfg.totalWeightLb / 2.0) * 0.45359237;
	double weight_n_side = mass_kg_side * g;
	double r_f = (cfg.frontWheelInches / 2.0) * inches_to_meters;
	double r_b = (cfg.rearWheelInches / 2.0) * inches_to_meters;

	// MA calculations using specific motor pulleys
	double R_f = cfg.frontPulleyTeeth / cfg.motorPulleyFront;
	double R_b = cfg.rearPulleyTeeth / cfg.motorPulleyRear;

	double K_sys = r_f / (cfg.gearboxReduction * R_f);
	double kt = 60.0 / (2.0 * PI * cfg.kv);
	double omega_no_load = (cfg.kv * cfg.v_batt * cfg.speedFactor) * (2.0 * PI / 60.0);
	double torque_stall = cfg.currentLimit * kt * cfg.drivetrainEfficiency;
	double Traction_Limit = cfg.mu * weight_n_side * K_sys;
	double Inertia_eq = mass_kg_side * pow(K_sys, 2);

	ofstream csvFile;
	if (!csvName.empty())
	{
		if (csvName.find(".csv") == string::npos) csvName += ".csv";
		csvFile.open(csvName);

		// --- DETAILED CONFIGURATION HEADER ---
		csvFile << "--- CONFIGURATION SPECS ---\n";
		csvFile << "Total Weight (lb)," << cfg.totalWeightLb << "\n";
		csvFile << "Battery Voltage (V)," << cfg.v_batt << "\n";
		csvFile << "Motor KV," << cfg.kv << "\n";
		csvFile << "Gearbox Reduction," << cfg.gearboxReduction << "\n";
		csvFile << "--------------------------,\n";
		csvFile << "Front Wheel Diameter (in)," << cfg.frontWheelInches << "\n";
		csvFile << "Front Pulley (Driven)," << cfg.frontPulleyTeeth << "\n";
		csvFile << "Motor Pulley (Front Driving)," << cfg.motorPulleyFront << "\n";
		csvFile << "--------------------------,\n";
		csvFile << "Rear Wheel Diameter (in)," << cfg.rearWheelInches << "\n";
		csvFile << "Rear Pulley (Driven)," << cfg.rearPulleyTeeth << "\n";
		csvFile << "Motor Pulley (Rear Driving)," << cfg.motorPulleyRear << "\n";
		csvFile << "--------------------------,\n";
		csvFile << "Current Limit (A)," << cfg.currentLimit << "\n";
		csvFile << "Coeff of Friction (mu)," << cfg.mu << "\n";
		csvFile << "Drivetrain Efficiency," << cfg.drivetrainEfficiency << "\n";
		csvFile << "Speed Factor," << cfg.speedFactor << "\n\n";

		csvFile << "Time(s),Speed(mph),Accel(g),Force(N),Torque(Nm),Status\n";
	}

	double time = 0.0, omega_m = 0.0, vel_m_s = 0.0;
	cout << "\nRunning Simulation..." << endl;
	cout << "Time | Speed(mph) | Accel(g) | Status" << endl;

	while (time <= cfg.simTime)
	{
		double torque_pot = torque_stall * (1.0 - (omega_m / omega_no_load));
		double torque_m = min(max(torque_pot, 0.0), Traction_Limit);

		// Using fmax(Inertia_eq, 0.00001) to prevent division by zero
		double current_accel_mps2 = (torque_m * K_sys) / fmax(Inertia_eq, 0.00001);
		double alpha_m = torque_m / fmax(Inertia_eq, 0.00001);

		if (fmod(time + 0.00001, 0.1) < cfg.dt)
		{
			string status = (torque_m >= Traction_Limit - 0.01) ? "LIMIT" : "MOTOR";

			// Console output for quick check
			cout << fixed << setprecision(2) << time << " | " << vel_m_s * 2.237 << " | " << current_accel_mps2 / g << " | " << status << endl;

			// CSV output
			if (csvFile.is_open())
			{
				csvFile << time << "," << vel_m_s * 2.237 << "," << current_accel_mps2 / g << "," << (torque_m / K_sys) << "," << torque_m << "," << status << "\n";
			}
		}

		omega_m += alpha_m * cfg.dt;
		vel_m_s = omega_m * K_sys;
		time += cfg.dt;
	}

	if (csvFile.is_open())
	{
		csvFile.close();
		cout << "\n[SUCCESS] Results saved to: " << csvName << endl;
	}
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

double getValueAfterColon(const string& line)
{
	size_t colonPos = line.find(':');
	if (colonPos != string::npos)
	{
		try
		{
			return stod(line.substr(colonPos + 1));
		}
		catch (...)
		{
			return 0.0;
		}
	}
	return 0.0;
}

/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


void DisplayConfig(const Config& cfg)
{
	cout << "\n==========================================" << endl;
	cout << "        LOADED CONFIGURATION DATA          " << endl;
	cout << "==========================================" << endl;
	cout << left << setw(35) << "Simulation Time:" << cfg.simTime << " s" << endl;
	cout << setw(35) << "Time Step:" << cfg.dt << " s" << endl;
	cout << "------------------------------------------" << endl;
	cout << setw(35) << "Gearbox Reduction:" << cfg.gearboxReduction << ":1" << endl;
	cout << setw(35) << "Front Wheel Diameter:" << cfg.frontWheelInches << " in" << endl;
	cout << setw(35) << "Front Pulley (Driven):" << cfg.frontPulleyTeeth << " teeth" << endl;
	cout << setw(35) << "Motor Pulley (Front):" << cfg.motorPulleyFront << " teeth" << endl;
	cout << "------------------------------------------" << endl;
	cout << setw(35) << "Rear Wheel Diameter:" << cfg.rearWheelInches << " in" << endl;
	cout << setw(35) << "Rear Pulley (Driven):" << cfg.rearPulleyTeeth << " teeth" << endl;
	cout << setw(35) << "Motor Pulley (Rear):" << cfg.motorPulleyRear << " teeth" << endl;
	cout << "------------------------------------------" << endl;
	cout << setw(35) << "Speed Factor:" << cfg.speedFactor << endl;
	cout << setw(35) << "Drivetrain Efficiency:" << cfg.drivetrainEfficiency << endl;
	cout << setw(35) << "Motor Kv:" << cfg.kv << " rpm/V" << endl;
	cout << setw(35) << "Current Limit:" << cfg.currentLimit << " Amps" << endl;
	cout << setw(35) << "Battery Voltage:" << cfg.v_batt << " V" << endl;

	cout << "------------------------------------------" << endl;
	cout << setw(35) << "Coeff. of Friction (mu):" << cfg.mu << endl;
	cout << setw(35) << "Total Weight:" << cfg.totalWeightLb << " lbs" << endl;
	cout << "==========================================\n" << endl;
}

