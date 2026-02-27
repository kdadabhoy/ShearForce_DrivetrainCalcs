/*
	This code was AI generated based on the derivations I provided it.
	I might go back and look at it later / fix it.

	But yeah :)

	I do not believe it accounts for inertias the same way I was
	accounting for them in my derivation... so that'll prob need to be fixed at
	some point... 


	TLDR: Simulation tab on that excel sheet is good enough imo
	
*/


#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <algorithm>
#include <cassert>
using namespace std;

int main()
{
	// --- 1. Physical Constants & Inputs ---
	const double dt = 0.002;
	const double total_weight_lb = 30.0;
	const double g = 9.80665;
	const double mu = 1.0;							// Coefficient of Friction

	// Per-side mass (assuming two identical drive systems)
	const double mass_kg_side = (total_weight_lb / 2.0) * 0.45359237;
	const double weight_n_side = mass_kg_side * g;




	// --- 2. Gearing & Geometry ---
	const double G_box = 19.0;				// Gearbox reduction

	// Wheel Radii
	const double r_f = 2.5 / 2 * 0.0254;    // Front Wheel Radius
	const double r_b = 3.5 / 2 * 0.0254;    // Rear Wheel Radius 


	// Pulley Teeth
	const double Motor_Pulley_Teeth_Front = 18.0;   // Motor Pulley Corresponding to front pulley
	const double Motor_Pulley_Teeth_Back = 18.0;	// Motor Pulley Corresponding to back pulley

	const double Front_Pulley_Teeth = 20.0;
	const double Rear_Pulley_Teeth = 28.0;



	// Speed Ratios (aka MA) = Teeth Driven / Teeth Driving 
	const double R_f = Front_Pulley_Teeth / Motor_Pulley_Teeth_Front; // Teeth Front Pulley / Teeth Motor Pulley (for front)
	const double R_b = Rear_Pulley_Teeth / Motor_Pulley_Teeth_Back;   // Teeth Back Pulley / Teeth Motor Pulley (for back)
	
	// Check to make sure r_f / R_f = r_b / R_b
	if ((r_f / R_f) != (r_b / R_b))
	{
		cout << "Pulley ratios do not work!" << endl;
		cout << (r_f / R_f) << " " << (r_b / R_b) << endl;
		cout << endl << endl;
		//assert((r_f / R_f) == (r_b / R_b));
	}


	// K_system = r_f / (G*R_f) = r_b / (G*R_b)
	const double K_sys = r_f / (G_box * R_f);



	// --- 3. Motor Specs (BadAss 3515-1130kv) ---
	const double speed_factor = .85;		// Accounting for air resistance/losses
	const double v_batt = 22.2;
	const double kv = 1130.0;
	const double kt = 9.549296 / kv;
	const double efficiency = 0.80;			// Drivetrain efficiency
	const double current_limit = 80.0;

	const double omega_no_load = (kv * v_batt * speed_factor) * (2.0 * 3.1415 / 60.0);
	const double torque_stall = current_limit * kt * efficiency;


	// --- 4. Traction & Inertia Derivations ---
	// Combined Traction: T_limit_total = mu*(Nf + Nr) * K_sys = mu*Mass_per_side*K_sys
	double Traction_Limit = mu * weight_n_side * K_sys;


	// Equivalent Inertia (Reflected Mass): Inertia_eq = m*K_sys^2 + I_fp / Rf^2 + I_bp / Rb^2 + I_m
	double Inertia_eq = mass_kg_side * std::pow(K_sys, 2); // ignores pulley inertias and motor inertia for now




	// --- 5. Simulation State ---
	double time = 0.0, dist_m = 0.0, vel_m_s = 0.0, omega_m = 0.0;

	std::cout << "Speed Factor: " << speed_factor << std::endl;
	std::cout << "Time | Speed (mph) | Accel (g) | Force per side (N) | M_Torque per side | Status" << std::endl;
	std::cout << "------------------------------------------------------------" << std::endl;

	while (time <= 5.0) // 3-second dash
	{
		// t-w curve torque (max possible torque from motor at this instance)
		double torque_pot = torque_stall * (1.0 - (omega_m / omega_no_load));

		// Piecewise torque picking
		double torque_m = min(max(torque_pot, 0.0), Traction_Limit);

		// Acceleration calcs
		double alpha_m = torque_m / Inertia_eq;
		double current_accel_mps2 = alpha_m * K_sys;

		// Time-step integration
		omega_m += alpha_m * dt;
		vel_m_s = omega_m * K_sys;
		dist_m += vel_m_s * dt;
		time += dt;

		if (std::fmod(time + 0.0001, 0.1) < dt)
		{
			std::string status = (torque_m >= Traction_Limit - 0.01) ? "LIMIT" : "MOTOR";
			double total_force = (torque_m / K_sys);

			std::cout << std::fixed << std::setprecision(2)
				<< time << " | "
				<< vel_m_s * 2.237 << "      | "
				<< current_accel_mps2 / g << "    | "
				<< total_force << "    | "
				<< torque_m << "    | " << status << std::endl;
		}
	}
	return 0;
}