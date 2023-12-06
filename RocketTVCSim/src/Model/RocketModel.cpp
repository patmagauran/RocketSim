#include "RocketModel.h"
#include "../Util/DataLog/DataLog.h"
#include "../Control/Thrust/ThrustParameters.h"


/*
Make 2 cylinders joined by a fixed link

*/


using namespace chrono;
using namespace chrono::irrlicht;

ChVector<> computeSingleInertia(double length, double radius, double mass) {
	return ChVector<double>((1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(length, 2)),
		0.5 * mass * pow(radius, 2),
		(1.0 / 12.0) * mass * (3 * pow(radius, 2) + pow(length, 2)));
}

double computeParallelAxisTheorem(double inertiaCenter, double distance, double mass) {
	return inertiaCenter + (mass * (pow(distance , 2)));
}
ChVector<> computeCompositeInertia(double lengthA, double LengthB, double massA, double massB, ChVector<> inertiaA, ChVector<> inertiaB) {
	
	double Ixx = computeParallelAxisTheorem(inertiaA.x(), lengthA / 2, massA) + computeParallelAxisTheorem(inertiaB.x(), LengthB / 2, massB);
	double Iyy = inertiaA.y() + inertiaB.y(); //Y is coincident for both
	double Izz = computeParallelAxisTheorem(inertiaA.z(), lengthA / 2, massA) + computeParallelAxisTheorem(inertiaB.z(), LengthB / 2, massB);
	return ChVector<>(Ixx, Iyy, Izz);
}

std::shared_ptr < ChBody > RocketModel::makeCylinder(double radius, double length, ChColor color, std::shared_ptr< ChMaterialSurface > material, double mass) {
	auto cylinder = std::make_shared<ChBody>();

	double density = mass / (CH_C_PI * pow(radius, 2) * length);
	cylinder->SetDensity(density);
	cylinder->SetMass(mass);
	cylinder->SetInertiaXX(computeSingleInertia(length, radius, mass));
	cylinder->GetCollisionModel()->ClearModel();
	cylinder->GetCollisionModel()->AddCylinder(material, radius, radius, length / 2);
	cylinder->GetCollisionModel()->BuildModel();
	cylinder->SetCollide(true);

	auto vshape = std::make_shared<ChCylinderShape>();
	vshape->GetGeometry().r = radius;
	vshape->GetGeometry().h = length;
	vshape->SetColor(color);

	auto vmodel = std::make_shared<ChVisualModel>();
	vmodel->AddShape(vshape, ChFrame<>(ChVector<>(0, 0, 0), ChQuaternion<>(0, 0, 0.7071068, 0.7071068)));
	cylinder->AddVisualModel(vmodel);

	return cylinder;
}

//I Should rewrite this to be cleaner
RocketModel::RocketModel(double rocket_radius, double lengthAG, double lengthGB, double rocket_mass, double maxThrustAngle, double maxRotationRate, double maxThrust) : maxThrustAngle(maxThrustAngle), maxRotationRate(maxRotationRate), maxThrust(maxThrust), m(rocket_mass), d_nozzle(lengthGB), T(maxThrust), inertia(computeCompositeInertia(lengthAG, lengthGB, rocket_mass / 2, rocket_mass / 2, computeSingleInertia(lengthAG, rocket_radius, rocket_mass / 2), computeSingleInertia(lengthGB, rocket_radius, rocket_mass / 2)))
{

	auto material = std::make_shared<ChMaterialSurfaceNSC>();
	double comp_mass = rocket_mass / 2;
	this->rocket_upper = makeCylinder(rocket_radius, lengthAG, ChColor(1, 1, 0.2), material, comp_mass);
	this->rocket_lower = makeCylinder(rocket_radius, lengthGB, ChColor(1, 1, 1), material, comp_mass);

	this->rocket_upper->SetPos(ChVector<>(0, lengthAG / 2, 0));
	this->rocket_upper->SetPos_dt(ChVector<>(0, 0, 0));

	this->rocket_lower->SetPos(ChVector<>(0, -lengthGB / 2, 0));
	this->rocket_lower->SetPos_dt(ChVector<>(0, 0, 0));
	this->fixed_link = std::make_shared<ChLinkMateFix>();

	this->fixed_link->Initialize(this->rocket_lower, this->rocket_upper, ChFrame<>(ChCoordsys<>(ChVector<>(0, 0, 0))));
	this->thrust_point = ChVector<>(0, -lengthGB / 2, 0);


}

RocketModel::RocketModel(RocketParams params) : RocketModel(params.getRocketRadius(), params.getLengthAG(), params.getLengthGB(), params.getRocketMass(), params.getMaxThrustAngle(), params.getMaxRotationRate(), params.getMaxThrust())
{
}

void RocketModel::reset(RocketParams params) {

	double rocket_radius;
	double lengthAG;
	double lengthGB;
	double rocket_mass;
	rocket_radius = params.getRocketRadius();
	lengthAG = params.getLengthAG();
	lengthGB = params.getLengthGB();
	rocket_mass = params.getRocketMass();
	maxThrustAngle = params.getMaxThrustAngle();
	maxRotationRate = params.getMaxRotationRate();
	maxThrust = params.getMaxThrust();
	m = rocket_mass;
	d_nozzle = lengthGB;
	gamma_1 = 0;
	gamma_2 = 0;
	T = maxThrust;


	this->rocket_lower.reset();
	this->rocket_upper.reset();
	this->fixed_link.reset();

	auto material = std::make_shared<ChMaterialSurfaceNSC>();
	double comp_mass = rocket_mass / 2;
	this->rocket_upper = makeCylinder(rocket_radius, lengthAG, ChColor(1, 1, 0.2), material, comp_mass);
	this->rocket_lower = makeCylinder(rocket_radius, lengthGB, ChColor(1, 1, 1), material, comp_mass);

	this->rocket_upper->SetPos(ChVector<>(0, lengthAG / 2, 0));
	this->rocket_upper->SetPos_dt(ChVector<>(0, 0, 0));
	this->rocket_lower->SetPos(ChVector<>(0, -lengthGB / 2, 0));
	this->rocket_lower->SetPos_dt(ChVector<>(0, 0, 0));
	this->fixed_link = std::make_shared<ChLinkMateFix>();
	this->fixed_link->Initialize(this->rocket_lower, this->rocket_upper, ChFrame<>(ChCoordsys<>(ChVector<>(0, 0, 0))));
	this->thrust_point = ChVector<>(0, -lengthGB / 2, 0);
}


RocketModel::~RocketModel()
{


}


void RocketModel::addRocketModelToSystem(chrono::ChSystem& system, chrono::ChVisualSystem& vis) {
	system.AddBody(this->rocket_lower);
	system.AddBody(this->rocket_upper);
	system.AddLink(this->fixed_link);
	vis.BindItem(this->rocket_lower);
	vis.BindItem(this->rocket_upper);

}
void RocketModel::accumulateDrag(ChVector<> wind) {

}
void RocketModel::accumulateForces(ThrustParameters thrustParams)
{

	ChVector<> thrust_force = thrustParams.convertToForceVector();

	//Need to verify these
	this->gamma_1 = thrustParams.pitchAngle;
	this->gamma_2 = thrustParams.yawAngle;
	this->T = thrustParams.force;
	this->forces.clear();
	this->rocket_lower->Empty_forces_accumulators();
	this->rocket_upper->Empty_forces_accumulators();

	this->rocket_lower->Accumulate_force(thrust_force, this->thrust_point, true);
	this->forces.push_back(ForceApplication(rocket_lower->Dir_Body2World(thrust_force), rocket_lower->Point_Body2World(this->thrust_point)));
}

std::list<ForceApplication> RocketModel::getDisplayedForces()
{
	return this->forces;
}

ChVector<> RocketModel::getGLocation()
{
	return this->fixed_link->GetLinkAbsoluteCoords().pos;
}

std::shared_ptr<ChBody> RocketModel::getRocketUpper()
{
	return this->rocket_upper;
}

double RocketModel::getMaxThrust()
{
	return maxThrust;
}

double RocketModel::getMaxThrustAngle()
{
	return this->maxThrustAngle;
}

double RocketModel::getMaxRotationRate()
{
	return this->maxRotationRate;
}

void RocketModel::logRocketData()
{
	DataLog::logData("rocket_x", this->getGLocation().x());
	DataLog::logData("rocket_y", this->getGLocation().y());
	DataLog::logData("rocket_z", this->getGLocation().z());
	DataLog::logData("rocket_vx", this->rocket_lower->GetPos_dt().x());
	DataLog::logData("rocket_vy", this->rocket_lower->GetPos_dt().y());
	DataLog::logData("rocket_vz", this->rocket_lower->GetPos_dt().z());
	DataLog::logData("rocket_ax", this->rocket_lower->GetPos_dtdt().x());
	DataLog::logData("rocket_ay", this->rocket_lower->GetPos_dtdt().y());
	DataLog::logData("rocket_az", this->rocket_lower->GetPos_dtdt().z());
	DataLog::logData("rocket_theta", this->rocket_lower->GetRot().Q_to_NasaAngles().x());
	DataLog::logData("rocket_phi", this->rocket_lower->GetRot().Q_to_NasaAngles().y());
	DataLog::logData("rocket_psi", this->rocket_lower->GetRot().Q_to_NasaAngles().z());
	DataLog::logData("rocket_pitch_rate", this->rocket_lower->GetWvel_loc().x());
	DataLog::logData("rocket_wy", this->rocket_lower->GetWvel_loc().y());
	DataLog::logData("rocket_yaw_rate", this->rocket_lower->GetWvel_loc().z());
	DataLog::logData("rocket_alphx", this->rocket_lower->GetWacc_loc().x());
	DataLog::logData("rocket_alphy", this->rocket_lower->GetWacc_loc().y());
	DataLog::logData("rocket_alphz", this->rocket_lower->GetWacc_loc().z());

}



Eigen::Matrix<double, 7, 7> RocketModel::getAmatrix()
{
	Eigen::Matrix<double, 7, 7> A;

	double C_n = this->C_n;
	double C_a = this->C_a;
	double S = this->S;
	double rho = this->rho;
	double d = this->d;

	//The SSM assumes that the X-axis is the longitudinal axis of the rocket, need to adjust to match simulated model
	ChVector<> lower_Inertia = this->rocket_lower->GetInertiaXX();
	ChVector<> upper_Inertia = this->rocket_upper->GetInertiaXX();
	double I_x = this->inertia.x();
	double I_y = this->inertia.y();
	double I_z = this->inertia.z();



	double v_x = this->rocket_lower->GetPos_dt().x();
	double v_y = this->rocket_lower->GetPos_dt().y();
	double v_z = this->rocket_lower->GetPos_dt().z();
	double w_x = this->rocket_lower->GetWvel_loc().x();
	double w_y = this->rocket_lower->GetWvel_loc().y();
	double w_z = this->rocket_lower->GetWvel_loc().z();
	double v = this->rocket_lower->GetPos_dt().Length();
	double e_x = this->rocket_lower->GetRot().Q_to_NasaAngles().x();
	double e_y = this->rocket_lower->GetRot().Q_to_NasaAngles().y();
	double e_z = this->rocket_lower->GetRot().Q_to_NasaAngles().z();
	double m = this->rocket_lower->GetMass();
	double D_n = C_n * S * rho;
	double D_a = C_a * S * rho;
	double D_nv = D_n * (pow(v, 2));
	double D_mn = D_n * d;
	double D_mnv = D_mn * pow(v, 2);
	double v_xz = sqrt(pow(v_x, 2) + pow(v_x, 2));
	A(0, 0) = -D_a * v_x / m;
	A(0, 1) = -D_a * v_y / m + w_z; A(0, 1) = -D_a * v_y / m + w_z;
	A(0, 2) = -D_a * v_z / m - w_y;
	A(0, 3) = -v_z;
	A(0, 4) = v_y;
	A(0, 5) = 0;
	A(0, 6) = 0;
	A(1, 0) = -w_z + (-D_n * pow(v_x, 2) / v_xz + D_nv * pow(v_x, 2) / (2 * pow(v_xz, 3)) - D_nv / (2 * v_xz)) / m;
	A(1, 1) = -D_n * v_x * v_y / (m * v_xz);
	A(1, 2) = w_x + (-D_n * v_x * v_z / v_xz + D_nv * v_x * v_z / (2 * pow(v_xz, 3))) / m;
	A(1, 3) = 0;
	A(1, 4) = -v_x;
	A(1, 5) = 0;
	A(1, 6) = 0;
	A(2, 0) = w_y + (D_n * v_x / v_xz - D_nv * v_x / (2 * pow(v_xz, 3))) / m;
	A(2, 1) = D_n * v_y / (m * v_xz) - w_x;
	A(2, 2) = (D_n * v_z / v_xz - D_nv * v_z / (2 * pow(v_xz, 3))) / m;
	A(2, 3) = v_x;
	A(2, 4) = 0;
	A(2, 5) = 0;
	A(2, 6) = 0;
	A(3, 0) = (D_mn * pow(v, 2) * pow(v_x, 2) / (2 * pow(v_xz, 3)) - D_mn * pow(v, 2) / (2 * v_xz) - D_mn * pow(v_x, 2) / v_xz) / I_y;
	A(3, 1) = -D_mn * v_x * v_y / (I_y * v_xz);
	A(3, 2) = (D_mn * pow(v, 2) * v_x * v_z / (2 * pow(v_xz, 3)) - D_mn * v_x * v_z / v_xz) / I_y;
	A(3, 3) = 0;
	A(3, 4) = -w_x * (I_x - I_z) / I_y;
	A(3, 5) = 0;
	A(3, 6) = 0;
	A(4, 0) = (-D_mn * pow(v, 2) * v_x / (2 * pow(v_xz, 3)) + D_mn * v_x / v_xz) / I_z;
	A(4, 1) = D_mn * v_y / (I_z * v_xz);
	A(4, 2) = (-D_mn * pow(v, 2) * v_z / (2 * pow(v_xz, 3)) + D_mn * v_z / v_xz) / I_z;
	A(4, 3) = -w_x * (-I_x + I_y) / I_z;
	A(4, 4) = 0;
	A(4, 5) = 0;
	A(4, 6) = 0;
	A(5, 0) = 0;
	A(5, 1) = 0;
	A(5, 2) = 0;
	A(5, 3) = 0;
	A(5, 4) = 0;
	A(5, 5) = cos(e_x);
	A(5, 6) = -sin(e_x);
	A(6, 0) = 0;
	A(6, 1) = 0;
	A(6, 2) = 0;
	A(6, 3) = 0;
	A(6, 4) = 0;
	A(6, 5) = sin(e_x) / cos(e_y);
	A(6, 6) = cos(e_x) / cos(e_y);

	return A;
}

Eigen::Matrix<double, 7, 3> RocketModel::getBmatrix()
{
	Eigen::Matrix<double, 7, 3> B;
	double T = this->T;
	double m = this->m;
	double d_nozzle = this->d_nozzle;
	double gamma_1 = this->gamma_1;
	double gamma_2 = this->gamma_2;
	double I_y = this->inertia.y();
	double I_z = this->inertia.z();


	B(0, 0) = -T * sin(gamma_1) * cos(gamma_2) / m;
	B(0, 1) = -T * sin(gamma_2) * cos(gamma_1) / m;
	B(0, 2) = cos(gamma_1) * cos(gamma_2) / m;
	B(1, 0) = -T * sin(gamma_1) * sin(gamma_2) / m;
	B(1, 1) = T * cos(gamma_1) * cos(gamma_2) / m;
	B(1, 2) = sin(gamma_2) * cos(gamma_1) / m;
	B(2, 0) = -T * cos(gamma_1) / m;
	B(2, 1) = 0;
	B(2, 2) = -sin(gamma_1) / m;
	B(3, 0) = -T * d_nozzle * sin(gamma_1) * sin(gamma_2) / I_y;
	B(3, 1) = T * d_nozzle * cos(gamma_1) * cos(gamma_2) / I_y;
	B(3, 2) = d_nozzle * sin(gamma_2) * cos(gamma_1) / I_y;
	B(4, 0) = -T * d_nozzle * cos(gamma_1) / I_z;
	B(4, 1) = 0;
	B(4, 2) = -d_nozzle * sin(gamma_1) / I_z;
	B(5, 0) = 0;
	B(5, 1) = 0;
	B(5, 2) = 0;
	B(6, 0) = 0;
	B(6, 1) = 0;
	B(6, 2) = 0;
	return B;
}

