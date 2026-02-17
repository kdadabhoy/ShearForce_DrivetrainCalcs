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

int main()
{
	// --- 1. Physical Constants & Inputs ---
	const double dt = 0.002;
	const double total_weight_lb = 30.0;
	const double g = 9.80665;
	const double mu = 1.0; // Coefficient of Friction

	// Per-side mass (assuming two identical drive systems)
	const double mass_kg_side = (total_weight_lb / 2.0) * 0.45359237;
	const double weight_n_side = mass_kg_side * g;

	// --- 2. Gearing & Geometry ---
	const double G_box = 19.0;          // Gearbox reduction
	const double r_f = 1.0 * 0.0254;    // Front Wheel Radius (1 inch)
	const double r_r = 1.75 * 0.0254;   // Rear Wheel Radius (1.75 inches)

	// Belt Ratios (R = Driven / Driving)
	const double R_f_belt = 20.0 / 49.0;
	const double R_r_belt = 30.0 / 42.0;

	// Total Reductions from Motor to Wheel
	const double R_f_total = G_box * R_f_belt; // 7.755
	const double R_r_total = G_box * R_r_belt; // 13.571

	// Kinematic Check: v = (omega_m / R) * r
	// K_sys is the meters of travel per radian of motor rotation.
	// Front: 0.0254 / 7.755 = 0.003275
	// Rear: 0.04445 / 13.571 = 0.003275
	// They are perfectly synchronized.
	const double K_sys = r_f / R_f_total;

	// --- 3. Motor Specs (BadAss 3515-1130kv) ---
	const double v_batt = 22.2;
	const double kv = 1130.0;
	const double kt = 9.549296 / kv;
	const double speed_factor = 0.90; // Accounting for air resistance/losses
	const double efficiency = 0.80;   // Drivetrain efficiency
	const double current_limit = 80.0;

	const double omega_no_load = (kv * v_batt * speed_factor) * (2.0 * 3.1415 / 60.0);
	const double torque_stall = current_limit * kt * efficiency;

	// --- 4. Traction & Inertia Derivations ---
	// Combined Traction: T_limit_total = (mu * N_f * r_f / R_f) + (mu * N_r * r_r / R_r)
	double limit_f_motor = (weight_n_side * 0.60 * mu * r_f) / R_f_total;
	double limit_r_motor = (weight_n_side * 0.40 * mu * r_r) / R_r_total;
	double combined_traction_limit = limit_f_motor + limit_r_motor;

	// Equivalent Inertia (Reflected Mass): J_eq = m * (r/R)^2
	double j_eq = mass_kg_side * std::pow(K_sys, 2);

	// --- 5. Simulation State ---
	double time = 0.0, dist_m = 0.0, vel_m_s = 0.0, omega_m = 0.0;

	std::cout << "Time | Speed(mph) | Accel(g) | Force(N) | M_Torque | Status" << std::endl;
	std::cout << "------------------------------------------------------------" << std::endl;

	while (time <= 3.0) // 3-second dash
	{
		// A. Linear Motor Torque Droop
		double torque_pot = torque_stall * (1.0 - (omega_m / omega_no_load));

		// B. Traction Constraint (Summing front and rear capability)
		double torque_m = std::min(std::max(torque_pot, 0.0), combined_traction_limit);

		// C. Dynamics (Rotational Newton's 2nd Law: alpha = T / J)
		double alpha_m = torque_m / j_eq;
		double current_accel_mps2 = alpha_m * K_sys;

		// D. Integration
		omega_m += alpha_m * dt;
		vel_m_s = omega_m * K_sys;
		dist_m += vel_m_s * dt;
		time += dt;

		if (std::fmod(time + 0.0001, 0.1) < dt)
		{
			std::string status = (torque_m >= combined_traction_limit - 0.01) ? "LIMIT" : "MOTOR";
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