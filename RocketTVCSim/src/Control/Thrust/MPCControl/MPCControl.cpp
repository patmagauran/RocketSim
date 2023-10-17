#include "MPCControl.h"
#include <mpc/LMPC.hpp>

#define M_PI 3.14159
MPCControl::MPCControl()
{
	//mpc::LParameters params;

	//params.alpha = 1.6;
	//params.rho = 1e-6;
	//params.eps_rel = 1e-4;
	//params.eps_abs = 1e-4;
	//params.eps_prim_inf = 1e-3;
	//params.eps_dual_inf = 1e-3;
	//params.time_limit = 0;
	//params.enable_warm_start = false;
	//params.verbose = false;
	//params.adaptive_rho = true;
	//params.polish = true;

	//lmpc.setOptimizerParameters(params);
	//lmpc.setLoggerLevel(mpc::Logger::log_level::NORMAL);

}

ControlSystemType MPCControl::getControlSystemType()
{
	return ControlSystemType();
}

double MPCControl::computeAngle()
{
    mpc::LMPC<> optsolver(
        Tnx, Tnu, Tndu, Tny,
        Tph, Tch);
    optsolver.setLoggerLevel(mpc::Logger::log_level::DEEP);

    mpc::mat<Tnx, Tnx> Ad;
    Ad << 1, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0,
        0, 1, 0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 0.1, 0, 0, 0,
        0.0488, 0, 0, 1, 0, 0, 0.0016, 0, 0, 0.0992, 0, 0,
        0, -0.0488, 0, 0, 1, 0, 0, -0.0016, 0, 0, 0.0992, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0.0992,
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        0.9734, 0, 0, 0, 0, 0, 0.0488, 0, 0, 0.9846, 0, 0,
        0, -0.9734, 0, 0, 0, 0, 0, -0.0488, 0, 0, 0.9846, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.9846;

    mpc::mat<Tnx, Tnu> Bd;
    Bd << 0, -0.0726, 0, 0.0726,
        -0.0726, 0, 0.0726, 0,
        -0.0152, 0.0152, -0.0152, 0.0152,
        0, -0.0006, -0.0000, 0.0006,
        0.0006, 0, -0.0006, 0,
        0.0106, 0.0106, 0.0106, 0.0106,
        0, -1.4512, 0, 1.4512,
        -1.4512, 0, 1.4512, 0,
        -0.3049, 0.3049, -0.3049, 0.3049,
        0, -0.0236, 0, 0.0236,
        0.0236, 0, -0.0236, 0,
        0.2107, 0.2107, 0.2107, 0.2107;

    mpc::mat<Tny, Tnx> Cd;
    Cd.setIdentity();

    mpc::mat<Tny, Tnu> Dd;
    Dd.setZero();

    optsolver.setStateSpaceModel(Ad, Bd, Cd);

    optsolver.setDisturbances(
        mpc::mat<Tnx, Tndu>::Zero(),
        mpc::mat<Tny, Tndu>::Zero());

    mpc::mat<Tnu, Tph> InputWMat, DeltaInputWMat;
    mpc::mat<Tny, Tph> OutputWMat;

    optsolver.setObjectiveWeights(OutputWMat, InputWMat, DeltaInputWMat);

    mpc::cvec<Tnu> InputW, DeltaInputW;
    mpc::cvec<Tny> OutputW;

    OutputW << 0, 0, 10, 10, 10, 10, 0, 0, 0, 5, 5, 5;
    InputW << 0.1, 0.1, 0.1, 0.1;
    DeltaInputW << 0, 0, 0, 0;

    optsolver.setObjectiveWeights(OutputW, InputW, DeltaInputW, { 0, Tph });

    mpc::mat<Tnx, Tph> xminmat, xmaxmat;
    mpc::mat<Tny, Tph> yminmat, ymaxmat;
    mpc::mat<Tnu, Tph> uminmat, umaxmat;

    xminmat.setZero();
    xmaxmat.setZero();
    yminmat.setZero();
    ymaxmat.setZero();
    uminmat.setZero();
    umaxmat.setZero();

    optsolver.setConstraints(xminmat, uminmat, yminmat, xmaxmat, umaxmat, ymaxmat);

    mpc::cvec<Tnx> xmin, xmax;
    xmin << -M_PI / 6, -M_PI / 6, -mpc::inf, -mpc::inf, -mpc::inf, -1,
        -mpc::inf, -mpc::inf, -mpc::inf, -mpc::inf, -mpc::inf, -mpc::inf;

    xmax << M_PI / 6, M_PI / 6, mpc::inf, mpc::inf, mpc::inf, mpc::inf,
        mpc::inf, mpc::inf, mpc::inf, mpc::inf, mpc::inf, mpc::inf;

    mpc::cvec<Tny> ymin, ymax;
    ymin.setOnes();
    ymin *= -mpc::inf;
    ymax.setOnes();
    ymax *= mpc::inf;

    mpc::cvec<Tnu> umin, umax;
    double u0 = 10.5916;
    umin << 9.6, 9.6, 9.6, 9.6;
    umin.array() -= u0;
    umax << 13, 13, 13, 13;
    umax.array() -= u0;

    optsolver.setConstraints(xmin, umin, ymin, xmax, umax, ymax, { 0, Tph });
    optsolver.setConstraints(xmin, umin, ymin, xmax, umax, ymax, { 0, 1 });

    optsolver.setScalarConstraint(-mpc::inf, mpc::inf, mpc::cvec<Tnx>::Ones(), mpc::cvec<Tnu>::Ones(), { -1, -1 });
    optsolver.setScalarConstraint(0, -mpc::inf, mpc::inf, mpc::cvec<Tnx>::Ones(), mpc::cvec<Tnu>::Ones());

    optsolver.setReferences(mpc::mat<Tny, Tph>::Zero(), mpc::mat<Tnu, Tph>::Zero(), mpc::mat<Tnu, Tph>::Zero());

    mpc::cvec<Tny> yRef;
    yRef << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    optsolver.setReferences(yRef, mpc::cvec<Tnu>::Zero(), mpc::cvec<Tnu>::Zero(), { 0, Tph });

    mpc::LParameters params;
    params.maximum_iteration = 250;
    optsolver.setOptimizerParameters(params);

    optsolver.setExogenuosInputs(mpc::mat<Tndu, Tph>::Zero());
    optsolver.setExogenuosInputs(mpc::cvec<Tndu>::Zero(), { 0, Tph });

    auto res = optsolver.step(mpc::cvec<Tnx>::Zero(), mpc::cvec<Tnu>::Zero());
    auto seq = optsolver.getOptimalSequence();
    (void)seq;

    mpc::cvec<4> testRes;
    testRes << -0.9916, 1.74839, -0.9916, 1.74839;
    res.cmd.isApprox(testRes, 1e-4);
	return 0.0;
}

void MPCControl::setParamsThrustAngleFromRate(PIDParams params)
{
}

void MPCControl::setParamsRateFromAngle(PIDParams params)
{
}

PIDParams MPCControl::getParamsThrustAngleFromRate()
{
	return PIDParams(0,0,0);
}

PIDParams MPCControl::getParamsRateFromAngle()
{
	return PIDParams(0, 0, 0);
}

double MPCControl::getYawRateFromAngleDeviation(double target, double current, double currentTime)
{
	return 0.0;
}

double MPCControl::getYawThrustAngleFromRateDeviation(double target, double current, double currentTime)
{
	return 0.0;
}

double MPCControl::getPitchRateFromAngleDeviation(double target, double current, double currentTime)
{
	return 0.0;
}

double MPCControl::getPitchThrustAngleFromRateDeviation(double target, double current, double currentTime)
{
	return 0.0;
}
